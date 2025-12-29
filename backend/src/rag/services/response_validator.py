"""Response validator to ensure quality and accuracy of agent responses."""

from typing import List
from ..models.agent_response import AgentResponse
from ..utils.validation import validate_agent_response


class ResponseValidator:
    """Validates agent responses for quality, accuracy, and proper formatting."""
    
    def __init__(self):
        """Initialize the response validator."""
        pass
    
    def validate_response(self, response: AgentResponse) -> bool:
        """Validate a response for quality and correctness."""
        # Use the basic validation from utils
        validation_errors = validate_agent_response(response)
        if validation_errors:
            print(f"Response validation failed: {', '.join(validation_errors)}")
            return False
        
        # Additional quality checks
        if not self._check_content_quality(response.content):
            return False
        
        if not self._check_source_relevance(response):
            return False
        
        return True
    
    def _check_content_quality(self, content: str) -> bool:
        """Check if the response content meets quality standards."""
        # Check if content is not empty
        if not content or len(content.strip()) == 0:
            return False
        
        # Check if content is too generic
        generic_indicators = [
            "I don't know", 
            "I cannot determine",
            "I'm sorry, but I can't",
            "No information provided"
        ]
        
        content_lower = content.lower()
        for indicator in generic_indicators:
            if indicator.lower() in content_lower:
                # This doesn't necessarily mean it's bad, just check if there's useful information
                continue
        
        # Content should have at least some substance
        if len(content) < 10:
            return False
        
        return True
    
    def _check_source_relevance(self, response: AgentResponse) -> bool:
        """Check if the sources in the response are relevant to the content."""
        if not response.sources:
            return True  # Not necessarily an error, depends on context
        
        # Check if sources have valid URLs
        for source in response.sources:
            if not source.source_url.startswith(("http://", "https://")):
                return False
        
        return True
    
    def validate_response_format(self, response: AgentResponse) -> bool:
        """Validate that the response format meets specifications."""
        validation_errors = validate_agent_response(response)
        return len(validation_errors) == 0
    
    def get_validation_report(self, response: AgentResponse) -> dict:
        """Generate a detailed validation report for a response."""
        basic_errors = validate_agent_response(response)
        
        report = {
            "basic_validation_passed": len(basic_errors) == 0,
            "basic_errors": basic_errors,
            "content_quality_check": self._check_content_quality(response.content),
            "source_relevance_check": self._check_source_relevance(response),
            "confidence_level": response.confidence,
            "sources_count": len(response.sources)
        }
        
        return report