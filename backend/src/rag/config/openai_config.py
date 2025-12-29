"""OpenAI Configuration for the RAG system using OpenRouter."""

import os
from dotenv import load_dotenv
from openai import AsyncOpenAI


load_dotenv()

# Load the OpenRouter API key from environment variables
router_api_key = os.getenv("OPENROUTER_API_KEY")

# Create the AsyncOpenAI provider with OpenRouter base URL
provider = AsyncOpenAI(
    api_key=router_api_key,
    base_url="https://openrouter.ai/api/v1"
)


class OpenAIChatCompletionsModel:
    """A wrapper class for OpenAI chat completions model."""

    def __init__(self, model: str, openai_client):
        """
        Initialize the model wrapper.

        Args:
            model: The model name to use
            openai_client: The OpenAI client instance
        """
        self.model = model
        self.openai_client = openai_client

    async def chat_completion(self, messages, **kwargs):
        """
        Perform a chat completion using the model.

        Args:
            messages: List of message dictionaries
            **kwargs: Additional parameters for the completion

        Returns:
            The chat completion response
        """
        return await self.openai_client.chat.completions.create(
            model=self.model,
            messages=messages,
            **kwargs
        )


class Agent:
    """A simple agent class that uses the OpenAI model."""

    def __init__(self, name: str, instructions: str, model):
        """
        Initialize the agent.

        Args:
            name: The name of the agent
            instructions: Instructions for the agent
            model: The model to use for completions
        """
        self.name = name
        self.instructions = instructions
        self.model = model

    async def run(self, user_input: str):
        """
        Run the agent with the given user input.

        Args:
            user_input: The input from the user

        Returns:
            The agent's response
        """
        messages = [
            {"role": "system", "content": self.instructions},
            {"role": "user", "content": user_input}
        ]

        response = await self.model.chat_completion(
            messages=messages,
            temperature=0.7
        )

        return response.choices[0].message.content


# Create the model instance
model = OpenAIChatCompletionsModel(
    model="mistralai/devstral-2512:free",
    openai_client=provider
)

# Create an example agent
agent_one = Agent(
    name="Frontend Expert",
    instructions="you are a frontend expert",
    model=model
)