from bs4 import BeautifulSoup
import re
from typing import List, Tuple

def clean_html_to_text(html_content: str) -> str:
    """
    Extract clean text content from HTML, removing navigation, headers, footers,
    and other non-content elements typical in Docusaurus sites
    """
    soup = BeautifulSoup(html_content, 'html.parser')
    
    # Remove common non-content elements
    for element in soup.find_all(['nav', 'header', 'footer', 'aside']):
        element.decompose()
    
    # Remove elements with classes that typically contain navigation
    for element in soup.find_all(class_=re.compile(r'(navbar|nav|header|footer|menu|sidebar|toc|pagination|footer|ads|advertisement)')):
        element.decompose()
    
    # Try to find the main content area - typical selectors for Docusaurus sites
    main_content = None
    content_selectors = [
        'main',
        'article',
        '.main-wrapper',
        '.theme-doc-markdown',
        '.markdown',
        '.doc-content',
        '.container',
        '[role="main"]'
    ]
    
    for selector in content_selectors:
        main_content = soup.select_one(selector)
        if main_content:
            break
    
    # If no specific content area found, use the body
    if not main_content:
        main_content = soup.find('body') or soup
    
    # Extract text and clean it up
    text = main_content.get_text(separator=' ')
    
    # Clean up extra whitespace
    text = re.sub(r'\s+', ' ', text).strip()
    
    return text

def extract_title_from_html(html_content: str) -> str:
    """
    Extract the page title from HTML content
    """
    soup = BeautifulSoup(html_content, 'html.parser')
    title_tag = soup.find('title')
    if title_tag:
        return title_tag.get_text().strip()
    
    # If no title tag, try to find an h1 element
    h1 = soup.find('h1')
    if h1:
        return h1.get_text().strip()
    
    return ""

def get_all_links_from_html(html_content: str, base_url: str) -> List[str]:
    """
    Extract all links from HTML content and return as absolute URLs
    """
    soup = BeautifulSoup(html_content, 'html.parser')
    links = []
    
    for link in soup.find_all('a', href=True):
        href = link['href']
        # Convert relative URLs to absolute
        if href.startswith('/'):
            href = base_url + href
        elif href.startswith('#') or href.startswith('mailto:') or href.startswith('tel:'):
            continue  # Skip anchors and non-HTTP links
        
        # Only add valid HTTP/HTTPS URLs
        if href.startswith(('http://', 'https://')):
            links.append(href)
    
    return links