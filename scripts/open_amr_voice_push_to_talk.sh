#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${AMR_DEVPC_CONTAINER:-amr_devpc}"
SESSION_NAME="${AMR_VOICE_PTT_SESSION:-amr_voice_push_to_talk}"
DEVICE="${AMR_VOICE_DEVICE:-8}"
ALSA_DEVICE="${AMR_VOICE_ALSA_DEVICE:-plughw:CARD=sofhdadsp,DEV=7}"
ASR_BACKEND="${AMR_ASR_BACKEND:-vosk}"
VOSK_MODEL="${AMR_VOSK_MODEL:-/workspaces/AMR-development/models/vosk-model-small-en-us-0.15}"
PIPER_BIN="${AMR_PIPER_BIN:-/workspaces/AMR-development/models/piper-runtime/piper}"
PIPER_MODEL="${AMR_PIPER_MODEL:-/workspaces/AMR-development/models/piper/en_US-lessac-medium/en_US-lessac-medium.onnx}"
TTS_OUTPUT_DEVICE="${AMR_TTS_OUTPUT_DEVICE:-plughw:CARD=sofhdadsp,DEV=0}"

if ! docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Container '${CONTAINER_NAME}' is not running. Start the AMR dev PC container first." >&2
  exit 1
fi

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required on the host." >&2
  exit 1
fi

if [ "${AMR_VOICE_CONFIGURE_AUDIO:-true}" = "true" ]; then
  AMR_DEVPC_CONTAINER="${CONTAINER_NAME}" ./scripts/configure_amr_voice_audio.sh >/tmp/amr_voice_audio_setup.log
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  tmux attach -t "${SESSION_NAME}"
  exit 0
fi

BASE_ENV='cd /workspaces/AMR-development/ros_ws && unset CYCLONEDDS_URI && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && source /opt/ros/foxy/setup.bash && source install/setup.bash'
TTS_ENV="AMR_PIPER_BIN=${PIPER_BIN} AMR_PIPER_MODEL=${PIPER_MODEL} AMR_TTS_OUTPUT_DEVICE=${TTS_OUTPUT_DEVICE} AMR_TTS_LENGTH_SCALE=${AMR_TTS_LENGTH_SCALE:-1.05} AMR_TTS_SENTENCE_SILENCE=${AMR_TTS_SENTENCE_SILENCE:-0.2} AMR_TTS_SPEAK_FEEDBACK=false"
CAPTURE_CMD="ros2 run amr_voice voice_pipeline_node --device ${DEVICE} --alsa-device ${ALSA_DEVICE} --asr-backend ${ASR_BACKEND} --vosk-model ${VOSK_MODEL} --start-listening --no-speech-timeout-sec 4 --max-utterance-sec 8 --duration-sec 10 --log-audio-level"

tmux new-session -d -s "${SESSION_NAME}" -n voice
tmux send-keys -t "${SESSION_NAME}:voice" \
  "docker exec -it ${CONTAINER_NAME} /entrypoint.sh bash -lc '${BASE_ENV} && ${TTS_ENV} ros2 run amr_voice tts_node'" C-m

tmux split-window -h -t "${SESSION_NAME}:voice"
tmux send-keys -t "${SESSION_NAME}:voice.1" \
  "docker exec -it ${CONTAINER_NAME} /entrypoint.sh bash -lc '${BASE_ENV} && ros2 run amr_voice conversation_runtime_node'" C-m

tmux split-window -v -t "${SESSION_NAME}:voice.1"
tmux send-keys -t "${SESSION_NAME}:voice.2" \
  "while true; do read -r -p 'Press Enter, speak one phrase, then pause. Type q + Enter to quit: ' answer; if [ \"\${answer}\" = q ]; then break; fi; docker exec -it ${CONTAINER_NAME} /entrypoint.sh bash -lc '${BASE_ENV} && ${CAPTURE_CMD}'; done" C-m

tmux select-layout -t "${SESSION_NAME}:voice" tiled >/dev/null
tmux attach -t "${SESSION_NAME}"
