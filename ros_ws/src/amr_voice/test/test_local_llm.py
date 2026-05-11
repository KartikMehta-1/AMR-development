import json
from unittest.mock import patch

from amr_voice.local_llm import LocalQwenHttpResponder, LocalQwenResponder, SYSTEM_PROMPT


def test_local_llm_safety_prompt_blocks_direct_runtime_changes():
    prompt = SYSTEM_PROMPT.lower()
    assert "never command motion" in prompt
    assert "clear faults" in prompt
    assert "supervised confirmation" in prompt


def test_extract_answer_removes_prompt_and_runtime_noise():
    prompt = "User question: What MCP servers do you have?\nAssistant answer:"
    output = f"""
Loading model
{prompt}
I can use AMR state inspection, mission control, robot launch, voice, speaker, and conversation MCP tools.
llama_perf_context_print:        load time = 1000 ms
"""

    answer = LocalQwenResponder._extract_answer(output, prompt)

    assert answer == (
        "I can use AMR state inspection, mission control, robot launch, voice, "
        "speaker, and conversation MCP tools."
    )


def test_extract_answer_handles_llama_prompt_truncation_banner():
    prompt = "User question: What MCP servers do you have?\nAssistant answer:"
    output = f"""
Loading model...
available commands:
  /exit or Ctrl+C     stop or exit

> {prompt}
Available AMR MCP servers:
- amr_state ... (truncated)

I have state inspection, mission control, robot launch, voice, speaker, and conversation MCP tools available.
common_memory_breakdown_print: | memory breakdown [MiB] |
"""

    answer = LocalQwenResponder._extract_answer(output, prompt)

    assert answer == (
        "I have state inspection, mission control, robot launch, voice, speaker, "
        "and conversation MCP tools available."
    )


def test_extract_answer_strips_inline_memory_breakdown_suffix():
    prompt = "User question: What can you do?\nAssistant answer:"
    output = (
        f"{prompt}\n"
        "I can answer questions and explain safe robot mission options."
        "common_memory_breakdown_print: | memory breakdown [MiB] |"
    )

    answer = LocalQwenResponder._extract_answer(output, prompt)

    assert answer == "I can answer questions and explain safe robot mission options."


class _FakeHttpResponse:
    def __init__(self, payload):
        self.payload = payload

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        return False

    def read(self):
        return json.dumps(self.payload).encode("utf-8")


def test_http_responder_reads_chat_completion_response():
    payload = {"choices": [{"message": {"content": "I can explain robot status."}}]}
    responder = LocalQwenHttpResponder(server_url="http://qwen.test/v1/chat/completions")

    with patch("urllib.request.urlopen", return_value=_FakeHttpResponse(payload)) as urlopen:
        result = responder.respond("what do you know about the robot")

    assert result.ok is True
    assert result.text == "I can explain robot status."
    request = urlopen.call_args.args[0]
    body = json.loads(request.data.decode("utf-8"))
    assert body["messages"][0]["role"] == "system"
    assert "Available AMR MCP servers" in body["messages"][1]["content"]
