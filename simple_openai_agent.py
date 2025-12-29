"""
Python Agent using OpenAI API with system and user messages for chat completion.
"""
import openai
import os
from typing import List, Dict, Any

# Configure OpenAI - get API key from environment variables
openai.api_key = os.getenv("OPENAI_API_KEY", "your_openai_api_key_here")

class SimpleChatAgent:
    def __init__(self, model: str = "gpt-4-turbo"):
        """
        Initialize the chat agent.

        Args:
            model: OpenAI model to use for completions
        """
        self.model = model
        self.conversation_history: List[Dict[str, str]] = []

    def chat(self, system_message: str, user_message: str) -> str:
        """
        Send a system message and user message to the agent and return the response.

        Args:
            system_message: System instruction/prompt for the agent
            user_message: User query/input

        Returns:
            Assistant response string
        """
        # Prepare the messages for the API call
        messages = [
            {"role": "system", "content": system_message},
            {"role": "user", "content": user_message}
        ]

        try:
            response = openai.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.7
            )
            assistant_reply = response.choices[0].message.content

            # Update conversation history
            self.conversation_history.extend([
                {"role": "system", "content": system_message},
                {"role": "user", "content": user_message},
                {"role": "assistant", "content": assistant_reply}
            ])

            return assistant_reply

        except Exception as e:
            return f"Error: {str(e)}"

    def chat_with_history(self, user_message: str) -> str:
        """
        Send a user message with conversation history context.

        Args:
            user_message: User query/input

        Returns:
            Assistant response string
        """
        # Add user message to history
        self.conversation_history.append({"role": "user", "content": user_message})

        try:
            response = openai.chat.completions.create(
                model=self.model,
                messages=self.conversation_history,
                temperature=0.7
            )
            assistant_reply = response.choices[0].message.content

            # Add assistant response to history
            self.conversation_history.append({"role": "assistant", "content": assistant_reply})

            return assistant_reply

        except Exception as e:
            return f"Error: {str(e)}"


def main():
    """Minimal working example of the chat agent."""
    # Initialize the agent
    agent = SimpleChatAgent()

    # Example 1: Basic system + user message
    system_msg = "You are a helpful assistant for a robotics textbook. Answer questions about humanoid robots, AI, and ROS 2."
    user_msg = "What is a humanoid robot?"

    response = agent.chat(system_msg, user_msg)
    print("Response:", response)

    # Example 2: Continuing conversation with history
    user_msg2 = "How does it differ from other types of robots?"
    response2 = agent.chat_with_history(user_msg2)
    print("Response 2:", response2)


if __name__ == "__main__":
    main()