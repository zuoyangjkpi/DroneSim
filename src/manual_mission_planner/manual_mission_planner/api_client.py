"""Client helpers for invoking the Qwen Omni Turbo API."""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from typing import Optional

import requests

DEFAULT_API_KEY = "sk-5db2e04d96f24a4bb2ccad84af9cdb4b"
API_URL = "https://dashscope.aliyuncs.com/api/v1/services/aigc/multimodal-generation/generation"


class QwenAPIError(RuntimeError):
    """Raised when the Qwen API request fails."""


@dataclass
class QwenMissionClient:
    """Thin wrapper around the Aliyun DashScope text-generation endpoint."""

    model: str = "qwen-omni-turbo"
    timeout: int = 30
    api_key: Optional[str] = None

    def __post_init__(self) -> None:
        env_key = os.environ.get("ALIYUN_API_KEY")
        if env_key:
            self.api_key = env_key
        elif not self.api_key:
            self.api_key = DEFAULT_API_KEY

    def generate_plan(self, system_prompt: str, user_prompt: str) -> str:
        """Call the LLM and return the raw assistant content."""

        payload = {
            "model": self.model,
            "input": {
                "messages": [
                    {
                        "role": "system",
                        "content": [
                            {"type": "text", "text": system_prompt},
                        ],
                    },
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": user_prompt},
                        ],
                    },
                ]
            },
            "parameters": {
                "result_format": "text",
                "temperature": 0.2,
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
            content = data["output"]["choices"][0]["message"]["content"]
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
