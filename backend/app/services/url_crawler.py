import asyncio
import time
import random
import aiohttp
from typing import List, Optional
from urllib.parse import urljoin, urlparse
import xml.etree.ElementTree as ET
from ..config.settings import settings

async def _get_all_urls_async(base_url: str) -> List[str]:
    """
    Discover and return all accessible URLs from a Docusaurus website
    Implements sitemap.xml discovery with fallback to link crawling
    """
    urls = set()

    # First try to get URLs from sitemap
    sitemap_url = base_url.rstrip('/') + '/sitemap.xml'
    sitemap_urls = await _get_urls_from_sitemap_async(sitemap_url)
    if sitemap_urls:
        urls.update(sitemap_urls)
    else:
        # Fallback to crawling the site
        urls.update(await _crawl_site_async(base_url))

    return list(urls)

def get_all_urls(base_url: str) -> List[str]:
    """
    Synchronous wrapper for the async function to get all URLs
    """
    return asyncio.run(_get_all_urls_async(base_url))

async def _get_urls_from_sitemap_async(sitemap_url: str) -> List[str]:
    """
    Extract URLs from a sitemap.xml file
    Implements retry logic with exponential backoff
    """
    last_exception = None

    for attempt in range(settings.MAX_RETRIES + 1):
        try:
            timeout = aiohttp.ClientTimeout(total=settings.TIMEOUT)
            async with aiohttp.ClientSession(timeout=timeout) as session:
                async with session.get(sitemap_url) as response:
                    if response.status == 200:
                        content = await response.text()
                        root = ET.fromstring(content)
                        # Find all <url><loc> elements
                        urls = []
                        for url_elem in root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}url'):
                            loc_elem = url_elem.find('{http://www.sitemaps.org/schemas/sitemap/0.9}loc')
                            if loc_elem is not None:
                                urls.append(loc_elem.text.strip())
                        return urls
                    else:
                        # If sitemap not found, return empty list
                        return []
        except Exception as e:
            last_exception = e
            if attempt < settings.MAX_RETRIES:
                # Exponential backoff with jitter
                delay = settings.DELAY_BASE * (2 ** attempt) + random.uniform(0, 1)
                await asyncio.sleep(delay)
            else:
                # All retry attempts failed, but we can continue with crawling
                print(f"Error getting sitemap: {str(e)}")
                return []

    return []

async def _crawl_site_async(base_url: str, max_depth: int = 2) -> List[str]:
    """
    Crawl the site to discover URLs (fallback method)
    Implements error handling
    """
    urls = set()
    to_visit = [(base_url, 0)]  # (url, depth)
    visited = set()

    timeout = aiohttp.ClientTimeout(total=settings.TIMEOUT)
    async with aiohttp.ClientSession(timeout=timeout) as session:
        while to_visit:
            current_url, depth = to_visit.pop(0)

            if current_url in visited or depth > max_depth:
                continue

            visited.add(current_url)

            try:
                async with session.get(current_url) as response:
                    if response.status == 200 and 'text/html' in response.content_type:
                        content = await response.text()
                        new_urls = _extract_links(content, base_url)

                        for url in new_urls:
                            if url not in visited and url.startswith(base_url):
                                to_visit.append((url, depth + 1))

                        urls.add(current_url)
            except Exception:
                # Skip URLs that can't be accessed
                continue

    return list(urls)

def _extract_links(html_content: str, base_url: str) -> List[str]:
    """
    Extract all links from HTML content and convert to absolute URLs
    """
    from bs4 import BeautifulSoup

    soup = BeautifulSoup(html_content, 'html.parser')
    links = []

    for link in soup.find_all('a', href=True):
        href = link['href']

        # Convert relative URLs to absolute
        if href.startswith('/'):
            href = urljoin(base_url, href)
        elif href.startswith('#') or href.startswith('mailto:') or href.startswith('tel:'):
            continue  # Skip anchors and non-HTTP links

        # Only add valid HTTP/HTTPS URLs from the same domain
        if href.startswith(('http://', 'https://')):
            parsed_href = urlparse(href)
            parsed_base = urlparse(base_url)
            if parsed_href.netloc == parsed_base.netloc:
                links.append(href)

    return links