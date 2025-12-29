import asyncio
import time
import random
import aiohttp
from typing import Dict, Any
from datetime import datetime
from ..utils.html_cleaner import clean_html_to_text, extract_title_from_html
from ..utils.id_generator import generate_page_hash
from ..config.settings import settings

async def _extract_text_from_url_async(url: str) -> Dict[str, Any]:
    """
    Extract clean text content from a single web page
    Implements retry logic with exponential backoff
    """
    last_exception = None

    for attempt in range(settings.MAX_RETRIES + 1):
        try:
            timeout = aiohttp.ClientTimeout(total=settings.TIMEOUT)
            async with aiohttp.ClientSession(timeout=timeout) as session:
                async with session.get(url) as response:
                    if response.status != 200:
                        raise Exception(f"Failed to retrieve URL: {url}, status code: {response.status}")

                    html_content = await response.text()
                    text_content = clean_html_to_text(html_content)
                    title = extract_title_from_html(html_content)
                    content_hash = generate_page_hash(html_content)

                    return {
                        'text': text_content,
                        'title': title,
                        'hash': content_hash,
                        'created_at': datetime.now().isoformat(),
                        'updated_at': datetime.now().isoformat()
                    }
        except Exception as e:
            last_exception = e
            if attempt < settings.MAX_RETRIES:
                # Exponential backoff with jitter
                delay = settings.DELAY_BASE * (2 ** attempt) + random.uniform(0, 1)
                await asyncio.sleep(delay)
            else:
                # All retry attempts failed
                raise Exception(f"Error extracting content from {url} after {settings.MAX_RETRIES} retries: {str(e)}")

    # This should not be reached, but included for completeness
    raise last_exception

def extract_text_from_url(url: str) -> Dict[str, Any]:
    """
    Synchronous wrapper for the async function to extract text from a URL
    """
    return asyncio.run(_extract_text_from_url_async(url))