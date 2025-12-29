import requests
from bs4 import BeautifulSoup
import re

# Test content extraction from one of the URLs
url = "https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app/docs/intro"

headers = {
    'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
}

response = requests.get(url, headers=headers, timeout=10)
print(f"Status code: {response.status_code}")
print(f"Response length: {len(response.text)} characters")

soup = BeautifulSoup(response.content, 'html.parser')

# Remove script and style elements
for script in soup(["script", "style"]):
    script.decompose()

# Remove navigation and UI elements
for element in soup(['nav', 'header', 'footer', 'aside']):
    element.decompose()
for element in soup.find_all(class_=re.compile(r'nav|menu|sidebar|footer|button|btn|navbar|breadcrumbs|toc|footer')):
    element.decompose()

# Try different selectors for main content
main_content_selectors = [
    'article',
    'main',
    'div[class*="theme-doc-markdown"]',
    'div[class*="container"]',
    'div[class*="content"]',
    'div[class*="docItem"]',
    'div[class*="markdown"]',
    '[role="main"]',
    '.main-wrapper',
    '.container',
    'article[class*="doc"]',
    'div[class*="doc"]'
]

print("\nTrying different selectors:")
for selector in main_content_selectors:
    element = soup.select_one(selector)
    if element:
        content = element.get_text(separator=' ', strip=True)
        print(f"Selector '{selector}': {len(content)} characters - Sample: {content[:100]}...")

# Get all text as fallback
all_text = soup.get_text(separator=' ', strip=True)
print(f"\nAll text fallback: {len(all_text)} characters - Sample: {all_text[:200]}...")

# Check for specific content areas that might contain the actual text
print("\nLooking for specific content elements...")
content_elements = soup.find_all(['p', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'li', 'td', 'th'])
content_text = ' '.join([elem.get_text().strip() for elem in content_elements if elem.get_text().strip()])
print(f"Content from text elements: {len(content_text)} characters - Sample: {content_text[:300]}...")

# Look for script tags that might contain JSON data or content
script_tags = soup.find_all('script')
print("\nLooking for script tags that might contain content...")
json_scripts = []
for script in script_tags:
    if script.get('type') == 'application/json' or 'props' in script.get('id', '') or 'data' in script.get('id', ''):
        content = script.get_text()
        if content and len(content) > 50:  # Only show substantial content
            json_scripts.append(content)
            print(f"JSON script: {len(content)} chars - Type: {script.get('type', 'no type')} - ID: {script.get('id', 'no id')}")
            print(f"  Sample: {content[:300]}...")

# Also look for any script tags that might contain content data
for i, script in enumerate(script_tags):
    script_content = script.get_text()
    if script_content and len(script_content) > 100 and ('{' in script_content or '[' in script_content):
        # This might be a JSON object with content
        print(f"Potential data script {i}: {len(script_content)} chars")
        print(f"  Type: {script.get('type', 'no type')}")
        print(f"  Sample: {script_content[:300]}...")
        if len(json_scripts) < 3:  # Don't print too many
            json_scripts.append(script_content)

# Look for content in data attributes or other areas
print("\nLooking for content in meta tags and other elements...")
meta_content = []
for meta in soup.find_all('meta'):
    if meta.get('name') or meta.get('property'):
        content = meta.get('content', '')
        if content and len(content) > 10:
            meta_content.append(content)
            print(f"Meta content: {content[:100]}...")

# Check for noscript tags
noscript_tags = soup.find_all('noscript')
if noscript_tags:
    print("\nFound noscript tags:")
    for noscript in noscript_tags:
        content = noscript.get_text().strip()
        print(f"Noscript content: {len(content)} chars - {content[:200]}...")