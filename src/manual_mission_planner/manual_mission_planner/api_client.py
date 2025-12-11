"""Client helpers for invoking the Qwen-compatible OpenAI chat API."""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import requests

# Default model and endpoint for Aliyun DashScope OpenAI-compatible API
DEFAULT_MODEL = "qwen-turbo"
MODEL_ENV_VAR = "QWEN_MODEL_NAME"
CONFIG_FILENAME = "llm_config.json"
CONFIG_ENV_VAR = "QWEN_CONFIG_FILE"
API_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"


class QwenAPIError(RuntimeError):
    """Raised when the Qwen API request fails."""


@dataclass
class QwenMissionClient:
    """Thin wrapper around the Aliyun DashScope OpenAI-compatible chat endpoint."""

    # You can change the default here, e.g. "qwen2.5-72b-instruct"
    model: str = DEFAULT_MODEL
    timeout: int = 30
    api_key: Optional[str] = None

    def __post_init__(self) -> None:
        # Optional: load API settings from a local JSON config file.
        #   1) By default, looks for llm_config.json next to this file.
        #   2) You can override the path with QWEN_CONFIG_FILE.
        config_path_env = os.environ.get(CONFIG_ENV_VAR)
        if config_path_env:
            config_path = Path(config_path_env).expanduser()
        else:
            config_path = Path(__file__).with_name(CONFIG_FILENAME)

        config: dict = {}
        if config_path.is_file():
            try:
                with config_path.open("r", encoding="utf-8") as f:
                    loaded = json.load(f)
                if isinstance(loaded, dict):
                    config = loaded
            except Exception:
                # If the config file is malformed, ignore it and fall back to
                # environment variables / constructor defaults.
                config = {}

        # Apply model from config unless an explicit env override is set.
        model_from_config = config.get("model")
        if model_from_config and not os.environ.get(MODEL_ENV_VAR):
            self.model = str(model_from_config)

        # Apply API key from config as a low-priority source.
        key_from_config = config.get("api_key")
        if key_from_config and not self.api_key:
            self.api_key = str(key_from_config)

        # Allow overriding the model from environment:
        #   export QWEN_MODEL_NAME="qwen2.5-72b-instruct"
        model_from_env = os.environ.get(MODEL_ENV_VAR)
        if model_from_env:
            self.model = model_from_env

        # Prefer explicit environment variables, fall back to any provided api_key.
        # Recommended: export DASHSCOPE_API_KEY="sk-xxx"
        env_key = (
            os.environ.get("DASHSCOPE_API_KEY")
            or os.environ.get("ALIYUN_API_KEY")
        )
        if env_key:
            self.api_key = env_key
        elif not self.api_key:
            raise QwenAPIError(
                "No API key provided. Set DASHSCOPE_API_KEY or ALIYUN_API_KEY."
            )

    def generate_plan(self, system_prompt: str, user_prompt: str) -> str:
        """Call the LLM and return the raw assistant content."""

        # OpenAI-compatible /v1/chat/completions payload
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ],
            # You can tweak extra_body if you want "thinking" traces, etc.
            "extra_body": {
                "enable_thinking": True,
            },
        }

        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}",
        }

        try:
            response = requests.post(
                API_URL,
                headers=headers,
                data=json.dumps(payload),
                timeout=self.timeout,
            )
        except requests.RequestException as exc:  # pylint: disable=broad-except
            raise QwenAPIError(f"Network error while calling Qwen API: {exc}") from exc
        if response.status_code != 200:
            raise QwenAPIError(
                f"HTTP {response.status_code}: {response.text[:200]}"
            )

        data = response.json()
        try:
            # OpenAI-compatible response shape:
            #   {"choices": [{"message": {"role": "...", "content": "..."}}], ...}
            content = data["choices"][0]["message"]["content"]
        except (KeyError, IndexError) as exc:
            raise QwenAPIError(f"Unexpected response: {data}") from exc

        if isinstance(content, list):
            joined = "\n".join(
                part.get("text", "") if isinstance(part, dict) else str(part)
                for part in content
            )
        else:
            joined = str(content)

        return self._extract_yaml_block(joined)

    @staticmethod
    def _extract_yaml_block(text: str) -> str:
        """Extract YAML content from the model response."""

        code_block = re.search(r"```(?:yaml)?\s*(.*?)```", text, flags=re.DOTALL | re.IGNORECASE)
        if code_block:
            return code_block.group(1).strip()
        return text.strip()


def build_fallback_plan(prompt: str) -> str:
    """Return a deterministic plan when the LLM is unavailable."""

    safe_name = prompt.strip().lower().replace(" ", "_")[:30] or "manual_test"
    return f"""mission:
  name: "fallback_{safe_name}"
  metadata:
    priority: 1
    time_budget: 300
    required_capabilities: [takeoff, search, track, land]
    version: "1.0.0"
    author: "fallback_planner"

parameters:
  target:
    class: "person"
    max_distance: 80.0

stages:
  initial: "takeoff"
  stage_list:
    - id: "takeoff"
      type: "TAKEOFF"
      params:
        target_altitude: 3.0
      transitions:
        success: "search"
        failure: "abort"
      timeout: 45

    - id: "search"
      type: "SEARCH_AREA"
      params:
        pattern: "rotate"
        duration: 120.0
        confirmations: 3
      transitions:
        target_found: "track"
        timeout: "land"
        failure: "abort"
      timeout: 120

    - id: "track"
      type: "TRACK_TARGET"
      params:
        target_class: "person"
        max_duration: 60.0
        min_duration: 5.0
        lose_timeout: 10.0
      transitions:
        success: "land"
        target_lost: "search"
        failure: "abort"

    - id: "land"
      type: "LAND_AT_POINT"
      params:
        target_position: [0.0, 0.0, 0.0]
        final_altitude: 0.05
      transitions:
        success: "complete"
        failure: "abort"
      timeout: 60

    - id: "abort"
      type: "ABORT_MISSION"
      params:
        reason: "mission_failed"

    - id: "complete"
      type: "TERMINAL"
""".strip()
