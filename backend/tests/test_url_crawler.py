import pytest
from unittest.mock import patch, MagicMock
from app.services.url_crawler import get_all_urls, _extract_links


class TestURLCrawler:
    """Unit tests for the URL crawler service"""
    
    def test_extract_links(self):
        """Test link extraction from HTML content"""
        html_content = """
        <html>
            <body>
                <a href="/page1">Page 1</a>
                <a href="https://example.com/page2">Page 2</a>
                <a href="#section1">Section</a>
                <a href="mailto:test@example.com">Email</a>
            </body>
        </html>
        """
        base_url = "https://example.com"
        links = _extract_links(html_content, base_url)
        
        # Should extract absolute URLs, ignore anchors and mailto
        expected_links = [
            "https://example.com/page1",  # Relative link converted to absolute
            "https://example.com/page2"  # Absolute link stays the same
        ]
        
        assert all(link in links for link in expected_links)
        assert len(links) == 2  # Should not include anchors or mailto