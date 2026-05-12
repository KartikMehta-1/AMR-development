from __future__ import annotations

import os
import re
import json
import subprocess
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Optional

from amr_voice.tts import sanitize_speech_text


DEFAULT_QWEN_MODEL_REF = "Qwen/Qwen2.5-7B-Instruct-GGUF:Q4_K_M"
DEFAULT_QWEN_MODEL_PATH = "models/qwen/qwen2.5-7b-instruct-q4_k_m-00001-of-00002.gguf"
DEFAULT_LLAMA_CLI = "models/llama.cpp/build/bin/llama-cli"
DEFAULT_QWEN_SERVER_URL = "http://127.0.0.1:8081/v1/chat/completions"


MCP_TOOL_SUMMARY = """Available AMR MCP servers:
- amr_state_inspection: read-only robot health, safety, localization, navigation, mission, and STM diagnostics.
- amr_mission_control: named-place mission readiness, status, place listing, and supervised go-to/cancel tools.
- amr_robot_launch: supervised robot launch and runtime bringup tools.
- amr_voice_interface: text intent parsing for voice transcripts.
- amr_speaker: text-to-speech requests through the speaker node.
- amr_conversation: one-turn conversation planning and safe tool routing.
"""


SYSTEM_PROMPT = """You are the AMR robot assistant.
Answer conversationally and briefly.
You may explain available MCP tools and robot capabilities.
Do not claim live robot state unless a read-only state tool result is provided.
Never command motion, clear faults, reset safety, re-enable hardware, launch robot runtime, or shut down machines directly.
For motion or runtime-changing requests, say that a supervised confirmation and the relevant MCP readiness checks are required.
"""


@dataclass(frozen=True)
class LocalLlmResult:
    ok: bool
    text: str
    message: str
    command: list[str]
    raw_output: str


class LocalQwenResponder:
    def __init__(
        self,
        llama_cli: str = DEFAULT_LLAMA_CLI,
        model_ref: str = DEFAULT_QWEN_MODEL_REF,
        model_path: str = DEFAULT_QWEN_MODEL_PATH,
        n_predict: int = 120,
        temperature: float = 0.4,
        timeout_sec: float = 90.0,
    ):
        self.llama_cli = llama_cli
        self.model_ref = model_ref
        self.model_path = model_path
        self.n_predict = n_predict
        self.temperature = temperature
        self.timeout_sec = timeout_sec

    @classmethod
    def from_env(cls) -> "LocalQwenResponder":
        return cls(
            llama_cli=os.environ.get("AMR_LLAMA_CLI", DEFAULT_LLAMA_CLI),
            model_ref=os.environ.get("AMR_QWEN_MODEL_REF", DEFAULT_QWEN_MODEL_REF),
            model_path=os.environ.get("AMR_QWEN_MODEL_PATH", DEFAULT_QWEN_MODEL_PATH),
            n_predict=int(os.environ.get("AMR_QWEN_N_PREDICT", "120")),
            temperature=float(os.environ.get("AMR_QWEN_TEMP", "0.4")),
            timeout_sec=float(os.environ.get("AMR_QWEN_TIMEOUT_SEC", "90")),
        )

    def respond(self, text: str, *, extra_context: Optional[str] = None) -> LocalLlmResult:
        prompt = self._prompt(text, extra_context=extra_context)
        command = [self.llama_cli]
        if self.model_path and os.path.exists(self.model_path):
            command.extend(["-m", self.model_path])
        else:
            command.extend(["-hf", self.model_ref])
        command.extend(
            [
                "-p",
                prompt,
                "-n",
                str(self.n_predict),
                "--temp",
                str(self.temperature),
                "--single-turn",
                "--simple-io",
                "--no-display-prompt",
                "--no-show-timings",
                "--no-warmup",
            ]
        )
        try:
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
                timeout=self.timeout_sec,
            )
        except FileNotFoundError:
            return LocalLlmResult(False, "", "llama-cli not found", command, "")
        except subprocess.TimeoutExpired as exc:
            return LocalLlmResult(False, "", "local LLM timed out", command, exc.stdout or "")
        if result.returncode != 0:
            return LocalLlmResult(False, "", f"local LLM failed with code {result.returncode}", command, result.stdout)
        answer = self._extract_answer(result.stdout, prompt)
        if not answer:
            return LocalLlmResult(False, "", "local LLM produced no answer", command, result.stdout)
        return LocalLlmResult(True, sanitize_speech_text(answer), "local LLM response", command, result.stdout)

    @staticmethod
    def _prompt(text: str, *, extra_context: Optional[str]) -> str:
        context = MCP_TOOL_SUMMARY
        if extra_context:
            context = f"{context}\nAdditional context:\n{extra_context.strip()}\n"
        return (
            f"{SYSTEM_PROMPT.strip()}\n\n"
            f"{context}\nUser question: {sanitize_speech_text(text)}\nAssistant answer:"
        )

    @staticmethod
    def _extract_answer(output: str, prompt: str) -> str:
        cleaned = output.replace("\b", "")
        for marker in ("common_memory_breakdown_print:", "llama_perf_", "[ Prompt:"):
            if marker in cleaned:
                cleaned = cleaned.split(marker, 1)[0]
        if "... (truncated)" in cleaned:
            cleaned = cleaned.split("... (truncated)", 1)[-1]
        if "Assistant answer:" in cleaned:
            cleaned = cleaned.rsplit("Assistant answer:", 1)[-1]
        elif prompt in cleaned:
            cleaned = cleaned.rsplit(prompt, 1)[-1]
        lines = []
        for line in cleaned.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.startswith(
                (
                    "Loading model",
                    "▄▄",
                    "██",
                    "build ",
                    "model ",
                    "modalities ",
                    "available commands",
                    ">",
                    "[ Prompt:",
                    "Exiting",
                    "common_memory_breakdown_print",
                    "llama_perf_",
                )
            ):
                continue
            lines.append(stripped)
        answer = " ".join(lines).strip()
        answer = re.sub(r"\s+", " ", answer)
        return answer


class LocalQwenHttpResponder:
    def __init__(
        self,
        server_url: str = DEFAULT_QWEN_SERVER_URL,
        model: str = "amr-qwen",
        n_predict: int = 120,
        temperature: float = 0.4,
        timeout_sec: float = 90.0,
    ):
        self.server_url = server_url
        self.model = model
        self.n_predict = n_predict
        self.temperature = temperature
        self.timeout_sec = timeout_sec

    @classmethod
    def from_env(cls) -> "LocalQwenHttpResponder":
        return cls(
            server_url=os.environ.get("AMR_QWEN_SERVER_URL", DEFAULT_QWEN_SERVER_URL),
            model=os.environ.get("AMR_QWEN_SERVER_MODEL", "amr-qwen"),
            n_predict=int(os.environ.get("AMR_QWEN_N_PREDICT", "120")),
            temperature=float(os.environ.get("AMR_QWEN_TEMP", "0.4")),
            timeout_sec=float(os.environ.get("AMR_QWEN_TIMEOUT_SEC", "90")),
        )

    def respond(self, text: str, *, extra_context: Optional[str] = None) -> LocalLlmResult:
        user_prompt = self._user_prompt(text, extra_context=extra_context)
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT.strip()},
                {"role": "user", "content": user_prompt},
            ],
            "temperature": self.temperature,
            "max_tokens": self.n_predict,
            "stream": False,
        }
        body = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            self.server_url,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                raw_output = response.read().decode("utf-8", errors="replace")
        except urllib.error.URLError as exc:
            return LocalLlmResult(False, "", f"local Qwen server unavailable: {exc}", ["POST", self.server_url], "")
        except TimeoutError:
            return LocalLlmResult(False, "", "local Qwen server timed out", ["POST", self.server_url], "")
        try:
            data = json.loads(raw_output)
            text_out = data["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError, json.JSONDecodeError) as exc:
            return LocalLlmResult(False, "", f"local Qwen server returned unexpected response: {exc}", ["POST", self.server_url], raw_output)
        answer = sanitize_speech_text(str(text_out))
        if not answer:
            return LocalLlmResult(False, "", "local Qwen server produced no answer", ["POST", self.server_url], raw_output)
        return LocalLlmResult(True, answer, "local Qwen server response", ["POST", self.server_url], raw_output)

    @staticmethod
    def _user_prompt(text: str, *, extra_context: Optional[str]) -> str:
        context = MCP_TOOL_SUMMARY
        if extra_context:
            context = f"{context}\nAdditional context:\n{extra_context.strip()}\n"
        return f"{context}\nUser question: {sanitize_speech_text(text)}"


def qwen_responder_from_env():
    server_url = os.environ.get("AMR_QWEN_SERVER_URL", "").strip()
    if server_url:
        return LocalQwenHttpResponder.from_env()
    return LocalQwenResponder.from_env()
