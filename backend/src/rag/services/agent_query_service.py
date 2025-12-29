"""Agent query service for processing user queries."""

import time
from typing import Dict, List, Optional
from ..models.query_request import QueryRequest
from ..models.agent_response import AgentResponse
from ..models.retrieved_content import RetrievedContent
from ..agent.openai_agent import OpenAIAgent
from ..agent.grounding_validator import GroundingValidator
from ..retrieval.retrieval_tool import RetrievalTool
from ..utils.helpers import calculate_elapsed_time, create_tool_call
from ..config.agent_config import AgentConfig
from ..constants import DEFAULT_GROUNDING_STRICTNESS
from ...config.settings import settings


def is_general_conversation(query: str) -> bool:
    """Check if the query is a general conversation rather than a textbook-related query."""
    query_lower = query.lower().strip()

    # List of common general conversation phrases
    general_phrases = [
        "hi", "hello", "hey", "greetings", "good morning", "good afternoon", "good evening",
        "how are you", "how's it going", "what's up", "how do you do", "nice to meet you",
        "what's your name", "who are you", "what can you do", "tell me about yourself",
        "goodbye", "bye", "see you", "farewell", "thanks", "thank you", "thx",
        "ok", "okay", "cool", "awesome", "great", "nice", "please"
    ]

    # Check if the query matches any general conversation phrases
    for phrase in general_phrases:
        if query_lower == phrase or query_lower.startswith(phrase) or phrase.startswith(query_lower):
            return True

    # Additional check for very short queries that might be greetings
    if len(query_lower.split()) <= 3:
        for word in query_lower.split():
            if word in ["hi", "hello", "hey", "bye", "goodbye", "thanks", "thank"]:
                return True

    return False


def is_book_topic_query(query: str) -> bool:
    """Check if the query is asking for book topics, names, meanings, or details."""
    query_lower = query.lower().strip()

    # Keywords that indicate the user wants to know about book topics
    topic_keywords = [
        "what is", "what's", "define", "definition", "meaning", "explain",
        "describe", "tell me about", "details about", "information about",
        "concept of", "topic", "subject", "chapter", "section", "module",
        "what does", "how does", "understand", "learn about", "topic on",
        "meaning of", "concept", "principle", "theory", "method", "approach",
        "system", "process", "technique", "framework", "algorithm"
    ]

    # Check if the query contains topic-related keywords
    for keyword in topic_keywords:
        if keyword in query_lower:
            return True

    # Additional check for queries that might be asking for specific information
    # like "What is ROS 2?" or "Explain digital twins"
    words = query_lower.split()
    if len(words) >= 2:
        # Look for patterns like "what is X", "explain X", "define X"
        if words[0] in ["what", "how", "explain", "define", "describe", "tell"] and \
           (words[1] in ["is", "does", "are", "can"] or words[0] in ["explain", "define", "describe", "tell"]):
            return True

    return False


class AgentQueryService:
    """Service to handle querying the agent with proper orchestration of tools and validation."""

    def __init__(self,
                 openai_agent: OpenAIAgent = None,
                 retrieval_tool: RetrievalTool = None,
                 grounding_validator: GroundingValidator = None):
        """Initialize the agent query service."""
        self.grounding_validator = grounding_validator or GroundingValidator(DEFAULT_GROUNDING_STRICTNESS)
        self.tool_calls_log = []

        # Initialize only OpenAI agent through OpenRouter
        if settings.openrouter_api_key and settings.openrouter_api_key != "your_openrouter_api_key_here" and len(settings.openrouter_api_key) > 20:
            # Initialize OpenAI agent with OpenRouter API key
            self.agent_type = "openai"
            from ..config.agent_config import AgentConfig
            agent_config = AgentConfig()
            try:
                self.openai_agent = OpenAIAgent(api_key=settings.openrouter_api_key, config=agent_config)
                self.openai_agent.setup_assistant()
            except Exception as e:
                print(f"Warning: Could not initialize OpenAI agent via OpenRouter: {e}")
                self.openai_agent = None
                self.agent_type = None
        else:
            # No valid OpenRouter API key - we'll handle queries with retrieval only
            self.agent_type = None
            self.openai_agent = None

        self.retrieval_tool = retrieval_tool

    def process_query(self, query_request: QueryRequest) -> AgentResponse:
        """Process a query and return a response from the agent."""
        start_time = time.time()

        try:
            # Validate the incoming query request
            from ..utils.validation import validate_query_request
            validation_errors = validate_query_request(query_request)
            if validation_errors:
                raise ValueError(f"Invalid query request: {', '.join(validation_errors)}")

            # Check if the query is general conversation
            if is_general_conversation(query_request.query_text):
                # Handle general conversation without requiring textbook content
                if self.agent_type == "openai" and self.openai_agent:
                    # Create a simple query for general conversation
                    agent_response = self.openai_agent.run_query(query_request)
                    query_time_ms = calculate_elapsed_time(start_time)

                    # Set a reasonable confidence for general conversation
                    agent_response.confidence = 0.8
                    agent_response.query_time_ms = query_time_ms
                    return agent_response
                else:
                    # No agent available - return a simple response for general conversation
                    query_time_ms = calculate_elapsed_time(start_time)
                    return AgentResponse(
                        response_id=f"resp_{int(time.time())}",
                        content="Hello! I'm the Physical AI & Humanoid Robotics textbook assistant. You can ask me questions about robotics, AI, ROS 2, or other topics from the textbook.",
                        sources=[],
                        confidence=0.8,
                        tool_calls=[],
                        created_at=time.time(),
                        query_time_ms=query_time_ms
                    )

            # Check if the query is asking for book topics, names, meanings, or details
            elif is_book_topic_query(query_request.query_text):
                # Handle book topic queries by retrieving relevant content and formatting appropriately
                # Determine retrieval parameters based on query and session config
                retrieval_top_k = getattr(query_request.response_options, 'retrieval_top_k', 5)
                similarity_threshold = getattr(query_request.response_options, 'similarity_threshold', 0.3)

                # Retrieve relevant content
                retrieved_content = self.retrieval_tool.retrieve_content(
                    query=query_request.query_text,
                    top_k=retrieval_top_k,
                    threshold=similarity_threshold
                )

                # Log the retrieval tool call
                retrieval_tool_call = create_tool_call(
                    "retrieval_tool",
                    {"query": query_request.query_text, "top_k": retrieval_top_k, "threshold": similarity_threshold},
                    f"Retrieved {len(retrieved_content)} content chunks for book topic query"
                )
                self.tool_calls_log.append(retrieval_tool_call)

                # Check if retrieval was successful
                if not retrieved_content:
                    # For book topic queries that don't match content, provide specific guidance
                    if self.agent_type == "openai" and self.openai_agent:
                        # Ask the agent to respond appropriately to the book topic query
                        fallback_query_request = QueryRequest(
                            query_text=f"The user asked for information about: '{query_request.query_text}'. I couldn't find relevant information in the textbook. Please acknowledge this and suggest related topics from the Physical AI & Humanoid Robotics textbook that might be helpful.",
                            session_id=query_request.session_id,
                            user_context=query_request.user_context,
                            response_options=query_request.response_options
                        )
                        agent_response = self.openai_agent.run_query(fallback_query_request)
                        query_time_ms = calculate_elapsed_time(start_time)
                        agent_response.query_time_ms = query_time_ms
                        return agent_response
                    else:
                        query_time_ms = calculate_elapsed_time(start_time)
                        return AgentResponse(
                            response_id=f"resp_{int(time.time())}",
                            content=f"I couldn't find specific information about '{query_request.query_text}' in the textbook. The textbook covers topics like ROS 2, Digital Twins, AI-Robot Brains, and Vision-Language-Action systems. Please try asking about these areas.",
                            sources=[],
                            confidence=0.0,
                            tool_calls=[retrieval_tool_call],
                            created_at=time.time(),
                            query_time_ms=query_time_ms
                        )

                # Prepare context from retrieved content for the agent
                # Limit context length to prevent exceeding query limits
                all_content = [chunk.content for chunk in retrieved_content]
                context = ""
                for content in all_content:
                    # Add content to context if it doesn't exceed reasonable limits
                    if len(context) + len(content) < 1500:  # Keep under 1500 chars to allow for query text
                        context += content + "\n\n"
                    else:
                        break  # Stop adding if we're approaching the limit

                # Run the query with the appropriate agent for book topic queries
                if self.agent_type == "openai" and self.openai_agent:
                    # Create a query request specifically for book topic with enhanced context
                    enhanced_query_request = QueryRequest(
                        query_text=f"Based strictly on the following context: {context}\n\nProvide a comprehensive explanation about: {query_request.query_text}. Include definitions, meanings, details, and examples if available in the context.",
                        session_id=query_request.session_id,
                        user_context=query_request.user_context,
                        response_options=query_request.response_options
                    )
                    agent_response = self.openai_agent.run_query(enhanced_query_request)
                else:
                    # No agent available - return retrieved content directly
                    query_time_ms = calculate_elapsed_time(start_time)
                    content = f"Here's information about {query_request.query_text} from the textbook:\n\n"
                    content += "\n\n".join([f"- {chunk.content}" for chunk in retrieved_content[:3]])  # Show top 3 results
                    content += "\n\nPlease provide valid API keys to get AI-generated answers based on this content."

                    return AgentResponse(
                        response_id=f"resp_{int(time.time())}",
                        content=content,
                        sources=[],
                        confidence=0.0,
                        tool_calls=[retrieval_tool_call],
                        created_at=time.time(),
                        query_time_ms=query_time_ms
                    )

                # Validate that the response is grounded in the retrieved content
                is_properly_grounded = self.grounding_validator.validate_response_quality(
                    agent_response,
                    retrieved_content
                )

                # If not properly grounded and fallback is enabled, try to improve
                if not is_properly_grounded:
                    # Create a more specific query with grounding instruction for book topics
                    enhanced_query = QueryRequest(
                        query_text=f"Based strictly on the following context: {context}\n\nProvide a comprehensive explanation about: {query_request.query_text}. Include definitions, meanings, details, and examples if available in the context. Make sure your response is directly based on the provided context.",
                        session_id=query_request.session_id,
                        user_context=query_request.user_context,
                        response_options=query_request.response_options
                    )

                    # Retry the query with enhanced context
                    if self.agent_type == "openai":
                        agent_response = self.openai_agent.run_query(enhanced_query)

                    # Re-validate
                    is_properly_grounded = self.grounding_validator.validate_response_quality(
                        agent_response,
                        retrieved_content
                    )

                # Calculate final confidence based on grounding validation
                if is_properly_grounded:
                    grounding_score = self.grounding_validator.calculate_grounding_score(agent_response, retrieved_content)
                    agent_response.confidence = grounding_score
                else:
                    # Reduce confidence if not properly grounded
                    agent_response.confidence *= 0.5  # Reduce by half if not properly grounded

                # Add the retrieval information to the response sources
                # For now, we just add the tool call - in a real implementation,
                # you'd want to add specific source references based on the retrieved content
                agent_response.tool_calls.extend([retrieval_tool_call])
                agent_response.query_time_ms = calculate_elapsed_time(start_time)

                return agent_response

            # Determine retrieval parameters based on query and session config
            retrieval_top_k = getattr(query_request.response_options, 'retrieval_top_k', 5)
            similarity_threshold = getattr(query_request.response_options, 'similarity_threshold', 0.3)

            # Retrieve relevant content
            retrieved_content = self.retrieval_tool.retrieve_content(
                query=query_request.query_text,
                top_k=retrieval_top_k,
                threshold=similarity_threshold
            )

            # Log the retrieval tool call
            retrieval_tool_call = create_tool_call(
                "retrieval_tool",
                {"query": query_request.query_text, "top_k": retrieval_top_k, "threshold": similarity_threshold},
                f"Retrieved {len(retrieved_content)} content chunks"
            )
            self.tool_calls_log.append(retrieval_tool_call)

            # Check if retrieval was successful
            if not retrieved_content:
                # For non-conversation queries that don't match textbook content, try to handle gracefully
                if self.agent_type == "openai" and self.openai_agent:
                    # Ask the agent to respond appropriately to the query
                    fallback_query_request = QueryRequest(
                        query_text=f"The user asked: '{query_request.query_text}'. I couldn't find relevant information in the textbook. Please acknowledge this and suggest they ask about robotics, AI, ROS 2, or other topics from the Physical AI & Humanoid Robotics textbook.",
                        session_id=query_request.session_id,
                        user_context=query_request.user_context,
                        response_options=query_request.response_options
                    )
                    agent_response = self.openai_agent.run_query(fallback_query_request)
                    query_time_ms = calculate_elapsed_time(start_time)
                    agent_response.query_time_ms = query_time_ms
                    return agent_response
                else:
                    query_time_ms = calculate_elapsed_time(start_time)
                    return AgentResponse(
                        response_id=f"resp_{int(time.time())}",
                        content="I couldn't find any relevant information to answer your question. I'm designed to help with questions about Physical AI & Humanoid Robotics, ROS 2, Digital Twins, AI-Robot Brains, and Vision-Language-Action systems. Please ask questions related to these topics from the textbook.",
                        sources=[],
                        confidence=0.0,
                        tool_calls=[retrieval_tool_call],
                        created_at=time.time(),
                        query_time_ms=query_time_ms
                    )

            # Prepare context from retrieved content for the agent
            # Limit context length to prevent exceeding query limits
            all_content = [chunk.content for chunk in retrieved_content]
            context = ""
            for content in all_content:
                # Add content to context if it doesn't exceed reasonable limits
                if len(context) + len(content) < 1500:  # Keep under 1500 chars to allow for query text
                    context += content + "\n\n"
                else:
                    break  # Stop adding if we're approaching the limit

            # Run the query with the appropriate agent
            if self.agent_type == "openai" and self.openai_agent:
                # Create a new query request with the context included
                enhanced_query_request = QueryRequest(
                    query_text=f"Based strictly on the following context: {context}\n\nAnswer this question: {query_request.query_text}",
                    session_id=query_request.session_id,
                    user_context=query_request.user_context,
                    response_options=query_request.response_options
                )
                agent_response = self.openai_agent.run_query(enhanced_query_request)
            else:
                # No agent available - return retrieved content directly
                query_time_ms = calculate_elapsed_time(start_time)
                content = "I found the following information in the textbook:\n\n"
                content += "\n\n".join([f"- {chunk.content}" for chunk in retrieved_content[:3]])  # Show top 3 results
                content += "\n\nPlease provide valid API keys to get AI-generated answers based on this content."

                return AgentResponse(
                    response_id=f"resp_{int(time.time())}",
                    content=content,
                    sources=[],
                    confidence=0.0,
                    tool_calls=[retrieval_tool_call],
                    created_at=time.time(),
                    query_time_ms=query_time_ms
                )

            # Validate that the response is grounded in the retrieved content
            is_properly_grounded = self.grounding_validator.validate_response_quality(
                agent_response,
                retrieved_content
            )

            # If not properly grounded and fallback is enabled, try to improve
            if not is_properly_grounded:
                # Create a more specific query with grounding instruction
                enhanced_query = QueryRequest(
                    query_text=f"Based strictly on the following context: {context}\n\nAnswer this question: {query_request.query_text}",
                    session_id=query_request.session_id,
                    user_context=query_request.user_context,
                    response_options=query_request.response_options
                )

                # Retry the query with enhanced context
                if self.agent_type == "openai":
                    agent_response = self.openai_agent.run_query(enhanced_query)

                # Re-validate
                is_properly_grounded = self.grounding_validator.validate_response_quality(
                    agent_response,
                    retrieved_content
                )

            # Calculate final confidence based on grounding validation
            if is_properly_grounded:
                grounding_score = self.grounding_validator.calculate_grounding_score(agent_response, retrieved_content)
                agent_response.confidence = grounding_score
            else:
                # Reduce confidence if not properly grounded
                agent_response.confidence *= 0.5  # Reduce by half if not properly grounded

            # Add the retrieval information to the response sources
            # For now, we just add the tool call - in a real implementation,
            # you'd want to add specific source references based on the retrieved content
            agent_response.tool_calls.extend([retrieval_tool_call])
            agent_response.query_time_ms = calculate_elapsed_time(start_time)

            return agent_response

        except Exception as e:
            # Handle errors gracefully
            query_time_ms = calculate_elapsed_time(start_time)
            error_tool_call = create_tool_call("error_handler", {"error": str(e)}, "Error handled during query processing")

            return AgentResponse(
                response_id=f"resp_error_{int(time.time())}",
                content=f"An error occurred while processing your query: {str(e)}",
                sources=[],
                confidence=0.0,
                tool_calls=[error_tool_call],
                created_at=time.time(),
                query_time_ms=query_time_ms
            )

    def process_session_query(self, query_request: QueryRequest, thread_id: Optional[str] = None) -> AgentResponse:
        """Process a query within a session context."""
        # This would be similar to process_query but keeping session context
        # For simplicity, reuse the same logic but pass the thread_id
        if thread_id:
            # TODO: Implement thread-specific logic
            pass
        return self.process_query(query_request)