"""Client helpers for invoking the Qwen-compatible OpenAI chat API."""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from openai import OpenAI

# Default model and endpoint for Aliyun DashScope OpenAI-compatible API
DEFAULT_MODEL = "qwen-turbo"
BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"


def _load_config() -> dict:
    """Load configuration from llm_config.json file."""
    config_dir = Path(__file__).parent
    config_path = config_dir / "llm_config.json"

    if not config_path.exists():
        return {}

    try:
        with open(config_path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        print(f"Warning: Failed to load llm_config.json: {e}")
        return {}


class QwenAPIError(RuntimeError):
    """Raised when the Qwen API request fails."""


@dataclass
class QwenMissionClient:
    """Thin wrapper around the Aliyun DashScope OpenAI-compatible chat endpoint."""

    model: str = DEFAULT_MODEL
    timeout: int = 30
    api_key: Optional[str] = None

    def __post_init__(self) -> None:
        # Load configuration from llm_config.json
        config = _load_config()

        # Get API key from config file or use the one provided in constructor
        if not self.api_key:
            self.api_key = (
                config.get("dashscope_api_key")
                or config.get("aliyun_api_key")
            )

        if not self.api_key:
            raise QwenAPIError(
                "No API key provided. Please create llm_config.json file "
                "at src/manual_mission_planner/manual_mission_planner/llm_config.json "
                "with 'dashscope_api_key' field. See llm_config.json.example for reference."
            )

        # Get model from config file if set
        model_from_config = config.get("model_name")
        if model_from_config:
            self.model = model_from_config

    def generate_plan(self, system_prompt: str, user_prompt: str) -> str:
        """Call the LLM with streaming and return the raw assistant content."""

        try:
            # Create OpenAI client with DashScope endpoint
            client = OpenAI(
                api_key=self.api_key,
                base_url=BASE_URL,
            )

            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ]

            # Use streaming with thinking enabled (but don't print the thinking process)
            completion = client.chat.completions.create(
                model=self.model,
                messages=messages,
                extra_body={"enable_thinking": True},
                stream=True
            )

            full_content = []  # Collect complete response content
            is_first_content = True

            for chunk in completion:
                delta = chunk.choices[0].delta

                # Skip reasoning_content (thinking process) - don't print it
                # Only collect and print the final content
                if hasattr(delta, "content") and delta.content:
                    if is_first_content:
                        print("\n" + "=" * 20 + "Generated Result" + "=" * 20)
                        is_first_content = False
                    print(delta.content, end="", flush=True)
                    full_content.append(delta.content)

            if full_content:
                print("\n" + "=" * 50 + "\n")

            # Merge complete content
            joined = "".join(full_content)
            return self._extract_yaml_block(joined)

        except Exception as exc:
            raise QwenAPIError(f"Error calling Qwen API: {exc}") from exc

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
