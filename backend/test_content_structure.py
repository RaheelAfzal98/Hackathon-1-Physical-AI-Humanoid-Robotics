import requests
from bs4 import BeautifulSoup
import re
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from webdriver_manager.chrome import ChromeDriverManager
from selenium.webdriver.chrome.service import Service
import time

# Test content structure from one of the URLs
url = "https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app/docs/intro"

# Use Selenium to render JavaScript content
chrome_options = Options()
chrome_options.add_argument("--headless")
chrome_options.add_argument("--no-sandbox")
chrome_options.add_argument("--disable-dev-shm-usage")
chrome_options.add_argument("--disable-gpu")
chrome_options.add_argument("--window-size=1920,1080")

service = Service(ChromeDriverManager().install())
driver = webdriver.Chrome(service=service, options=chrome_options)

try:
    driver.get(url)
    # Wait for the page to load and content to be rendered
    WebDriverWait(driver, 10).until(
        EC.presence_of_element_located((By.TAG_NAME, "body"))
    )

    # Wait a bit more for dynamic content to load
    time.sleep(5)

    # Get the page source after JavaScript execution
    html_content = driver.page_source

    # Parse with BeautifulSoup
    soup = BeautifulSoup(html_content, 'html.parser')

    print(f"Total HTML length: {len(html_content)}")

    # Look for all text elements that might contain the actual content
    print("\nLooking for content elements:")

    # Check for all paragraph tags
    paragraphs = soup.find_all('p')
    print(f"Found {len(paragraphs)} paragraph tags")
    for i, p in enumerate(paragraphs[:5]):  # Show first 5 paragraphs
        text = p.get_text().strip()
        if text and len(text) > 20:  # Only show substantial content
            print(f"  P {i+1}: {text[:100]}...")

    # Check for all headings
    headings = soup.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6'])
    print(f"\nFound {len(headings)} heading tags")
    for i, h in enumerate(headings[:10]):  # Show first 10 headings
        text = h.get_text().strip()
        print(f"  H{i+1}: {text}")

    # Check for divs with specific classes that might contain content
    content_divs = soup.find_all('div', class_=re.compile(r'theme|doc|content|markdown|container|main', re.I))
    print(f"\nFound {len(content_divs)} content-related divs")
    for i, div in enumerate(content_divs[:10]):
        text = div.get_text().strip()
        if text and len(text) > 50:  # Only show substantial content
            print(f"  Div {i+1} classes: {div.get('class', [])}")
            print(f"    Content: {text[:200]}...")

    # Get all text and see what's actually there
    all_text = soup.get_text(separator=' ', strip=True)
    print(f"\nTotal text length: {len(all_text)}")
    print(f"First 500 chars of all text: {all_text[:500]}...")

    # Look for script tags that might contain JSON content
    script_tags = soup.find_all('script')
    json_scripts = []
    for script in script_tags:
        if script.get('type') == 'application/json' or 'props' in script.get('id', '') or script.get('type', '').startswith('application/json'):
            content = script.get_text()
            if content and len(content) > 50:
                json_scripts.append(content)
                print(f"\nJSON script found: {len(content)} chars")
                print(f"  Content: {content[:300]}...")

    # Check for data attributes or content in other elements
    print("\nLooking for content in other elements:")
    for tag in soup.find_all():
        if tag.name not in ['script', 'style', 'meta', 'link', 'title']:
            content = tag.get_text().strip()
            if len(content) > 100 and len(content) < 1000:  # Medium-length content
                print(f"  {tag.name} tag with class {tag.get('class', 'no class')}: {content[:100]}...")

finally:
    driver.quit()