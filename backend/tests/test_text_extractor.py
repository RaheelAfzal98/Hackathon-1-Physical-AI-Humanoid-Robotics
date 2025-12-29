import pytest
from unittest.mock import patch, MagicMock
from app.services.text_extractor import extract_text_from_url


class TestTextExtractor:
    """Unit tests for the text extractor service"""
    
    @patch('app.services.text_extractor.aiohttp.ClientSession')
    def test_extract_text_from_url(self, mock_session_class):
        """Test extracting text from a URL"""
        # Mock the aiohttp session and response
        mock_session = MagicMock()
        mock_session_class.return_value.__aenter__.return_value = mock_session
        mock_get = MagicMock()
        mock_session.get.return_value.__aenter__.return_value = mock_get
        mock_get.status = 200
        mock_get.text.return_value = "<html><body><p>Sample text content</p></body></html>"
        
        # Call the function
        result = extract_text_from_url("https://example.com")
        
        # Verify the result
        assert "text" in result
        assert "title" in result
        assert "hash" in result
        assert "Sample text content" in result['text']