"""OpenAI Agent integration for the RAG system (using OpenRouter)."""

import asyncio
import time
from datetime import datetime
from typing import Any, Dict, List, Optional
import openai
from ..config.agent_config import AgentConfig
from ..models.query_request import QueryRequest
from ..models.agent_response import AgentResponse, SourceReference, ToolCall
from ..utils.helpers import get_timestamp, calculate_elapsed_time, create_tool_call, generate_response_id
from ..utils.validation import validate_agent_response
from ..constants import DEFAULT_OPENAI_MODEL


class OpenAIAgent:
    """Class to handle interactions with OpenAI-compatible API via OpenRouter."""

    def __init__(self, api_key: str, config: Optional[AgentConfig] = None, base_url: str = "https://openrouter.ai/api/v1"):
        """Initialize the OpenAI Agent with OpenRouter."""
        self.config = config or AgentConfig()
        self.client = openai.OpenAI(
            api_key=api_key,
            base_url=base_url
        )
        self.model = "mistralai/devstral-2512:free"  # Using the provided model

    def setup_assistant(self, model: str = DEFAULT_OPENAI_MODEL, name: str = "RAG Assistant",
                       instructions: str = None):
        """Set up the model configuration (no assistant creation needed for chat completions)."""
        if instructions is None:
            self.instructions = """You are an AI assistant for a robotics textbook. Answer questions based on the provided context.
            Always cite sources when providing answers. Ground all responses strictly in the retrieved content.
            For general conversation (like greetings), respond naturally and mention you can help with textbook topics.
            When users ask for definitions, meanings, or details about book topics, provide clear, comprehensive explanations
            based on the textbook content with examples where available."""
        else:
            self.instructions = instructions

    def run_query(self, query_request: QueryRequest) -> AgentResponse:
        """Process a query and return an agent response."""
        start_time = time.time()

        try:
            # Prepare the messages for the chat completion
            messages = [
                {"role": "system", "content": self.instructions},
                {"role": "user", "content": query_request.query_text}
            ]

            # Call the chat completions API
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=self.config.temperature if hasattr(self.config, 'temperature') else 0.7
            )

            # Extract the response content
            response_content = response.choices[0].message.content

            # Calculate query time
            query_time_ms = calculate_elapsed_time(start_time)
            response_id = generate_response_id()

            # Create the response object
            agent_response = AgentResponse(
                response_id=response_id,
                content=response_content,
                sources=[],
                confidence=0.75,  # Placeholder confidence
                tool_calls=[],
                created_at=get_timestamp(),
                query_time_ms=query_time_ms
            )

            # Validate the response
            validation_errors = validate_agent_response(agent_response)
            if validation_errors:
                print(f"Warning: Response validation issues: {'; '.join(validation_errors)}")

            return agent_response

        except Exception as e:
            # Handle errors gracefully
            query_time_ms = calculate_elapsed_time(start_time)
            error_response = AgentResponse(
                response_id=generate_response_id(),
                content=f"Error processing your query: {str(e)}",
                sources=[],
                confidence=0.0,
                tool_calls=[create_tool_call("error_handler", {"error": str(e)}, "Error processed")],
                created_at=get_timestamp(),
                query_time_ms=query_time_ms
            )

            return error_response