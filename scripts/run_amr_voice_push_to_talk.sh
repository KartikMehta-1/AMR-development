#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${AMR_DEVPC_CONTAINER:-amr_devpc}"
DEVICE="${AMR_VOICE_DEVICE:-8}"
ALSA_DEVICE="${AMR_VOICE_ALSA_DEVICE:-plughw:CARD=sofhdadsp,DEV=7}"
ASR_BACKEND="${AMR_ASR_BACKEND:-vosk}"
VOSK_MODEL="${AMR_VOSK_MODEL:-/workspaces/AMR-development/models/vosk-model-small-en-us-0.15}"
FASTER_WHISPER_MODEL="${AMR_FASTER_WHISPER_MODEL_DIR:-/workspaces/AMR-development/models/faster-whisper/small.en}"
PIPER_BIN="${AMR_PIPER_BIN:-/workspaces/AMR-development/models/piper-runtime/piper}"
PIPER_MODEL="${AMR_PIPER_MODEL:-/workspaces/AMR-development/models/piper/en_US-lessac-medium/en_US-lessac-medium.onnx}"
TTS_OUTPUT_DEVICE="${AMR_TTS_OUTPUT_DEVICE:-plughw:CARD=sofhdadsp,DEV=0}"
LLAMA_BIN="${AMR_LLAMA_CLI:-/workspaces/AMR-development/models/llama.cpp/build-foxy/bin/llama-cli}"
QWEN_MODEL_PATH="${AMR_QWEN_MODEL_PATH:-/workspaces/AMR-development/models/qwen/qwen2.5-7b-instruct-q4_k_m-00001-of-00002.gguf}"
ENABLE_LLM="${AMR_CONVERSATION_ENABLE_LLM:-true}"

if ! docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Container '${CONTAINER_NAME}' is not running. Start the AMR dev PC container first." >&2
  exit 1
fi

if [ "${AMR_VOICE_CONFIGURE_AUDIO:-true}" = "true" ]; then
  AMR_DEVPC_CONTAINER="${CONTAINER_NAME}" ./scripts/configure_amr_voice_audio.sh >/tmp/amr_voice_audio_setup.log
fi

BASE_ENV='cd /workspaces/AMR-development/ros_ws && unset CYCLONEDDS_URI && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && source /opt/ros/foxy/setup.bash && source install/setup.bash'
TTS_ENV="AMR_PIPER_BIN=${PIPER_BIN} AMR_PIPER_MODEL=${PIPER_MODEL} AMR_TTS_OUTPUT_DEVICE=${TTS_OUTPUT_DEVICE} AMR_TTS_LENGTH_SCALE=${AMR_TTS_LENGTH_SCALE:-1.05} AMR_TTS_SENTENCE_SILENCE=${AMR_TTS_SENTENCE_SILENCE:-0.2} AMR_TTS_SPEAK_FEEDBACK=false"
NO_SPEECH_TIMEOUT_SEC="${AMR_VOICE_NO_SPEECH_TIMEOUT_SEC:-4}"
MAX_UTTERANCE_SEC="${AMR_VOICE_MAX_UTTERANCE_SEC:-8}"
DURATION_SEC="${AMR_VOICE_DURATION_SEC:-10}"
CAPTURE_CMD="ros2 run amr_voice voice_pipeline_node --device ${DEVICE} --alsa-device ${ALSA_DEVICE} --asr-backend ${ASR_BACKEND} --vosk-model ${VOSK_MODEL} --faster-whisper-model ${FASTER_WHISPER_MODEL} --start-listening --no-speech-timeout-sec ${NO_SPEECH_TIMEOUT_SEC} --max-utterance-sec ${MAX_UTTERANCE_SEC} --duration-sec ${DURATION_SEC} --log-audio-level"

cleanup() {
  if [ -n "${TTS_PID:-}" ]; then kill "${TTS_PID}" >/dev/null 2>&1 || true; fi
  if [ -n "${CONV_PID:-}" ]; then kill "${CONV_PID}" >/dev/null 2>&1 || true; fi
  docker exec "${CONTAINER_NAME}" bash -lc "pkill -f 'tts_node' || true; pkill -f 'conversation_runtime_node' || true; pkill -f 'voice_pipeline_node' || true; pkill -f 'arecord' || true; pkill -f 'piper' || true; pkill -f 'aplay' || true" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

docker exec "${CONTAINER_NAME}" /entrypoint.sh bash -lc "${BASE_ENV} && ${TTS_ENV} ros2 run amr_voice tts_node" >/tmp/amr_voice_tts.log 2>&1 &
TTS_PID=$!
docker exec "${CONTAINER_NAME}" /entrypoint.sh bash -lc "${BASE_ENV} && AMR_LLAMA_CLI=${LLAMA_BIN} AMR_QWEN_MODEL_PATH=${QWEN_MODEL_PATH} AMR_QWEN_N_PREDICT=${AMR_QWEN_N_PREDICT:-90} AMR_QWEN_TIMEOUT_SEC=${AMR_QWEN_TIMEOUT_SEC:-90} AMR_CONVERSATION_ENABLE_LLM=${ENABLE_LLM} ros2 run amr_voice conversation_runtime_node" >/tmp/amr_voice_conversation.log 2>&1 &
CONV_PID=$!

sleep 2
echo "AMR voice push-to-talk is ready."
echo "Press Enter, say one short phrase, then pause. Type q then Enter to quit."
echo "Logs: /tmp/amr_voice_tts.log and /tmp/amr_voice_conversation.log"

while true; do
  read -r -p "> " answer
  if [ "${answer}" = "q" ]; then
    break
  fi
  docker exec -it "${CONTAINER_NAME}" /entrypoint.sh bash -lc "${BASE_ENV} && ${CAPTURE_CMD}"
done
