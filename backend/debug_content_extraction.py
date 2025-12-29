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

# Test the actual extraction process for a specific URL
url = "https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app/docs/intro"

print("Testing content extraction for:", url)

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

    # Remove script and style elements
    for script in soup(["script", "style"]):
        script.decompose()

    # Remove navigation and UI elements first
    for element in soup(['nav', 'header', 'footer', 'aside']):
        element.decompose()
    for element in soup.find_all(class_=re.compile(r'nav|menu|sidebar|footer|button|btn|navbar|breadcrumbs|toc|footer')):
        element.decompose()
    for element in soup.find_all(class_=re.compile(r'sidebar|nav|menu|footer|button|btn|navbar|toc')):
        element.decompose()

    print("After removing navigation elements, HTML length:", len(str(soup)))

    # Extract main content - try the same selectors as our code
    main_content = (
        soup.find('article', class_=lambda x: x and 'theme-doc-markdown' in x) or
        soup.find('div', class_=lambda x: x and 'container' in x) or
        soup.find('main') or
        soup.find('article') or
        soup.find('div', class_=lambda x: x and ('content' in x.lower() or 'docItem' in x or 'markdown' in x)) or
        soup.find('div', {'role': 'main'}) or  # Common role for main content
        soup.find('div', class_=lambda x: x and 'docItem' in x) or  # Docusaurus doc item
        soup.find('div', class_=lambda x: x and 'theme' in x and 'doc' in x) or  # Theme doc containers
        soup.find('div', class_=lambda x: x and 'docs' in x) or  # Docs containers
        soup
    )

    print(f"Found main_content: {main_content is not None}")
    if main_content:
        print(f"Main content tag: {main_content.name}")
        print(f"Main content classes: {main_content.get('class', [])}")

        # Get all text content
        all_content = main_content.get_text(separator=' ', strip=True)
        print(f"Text content length from main_content: {len(all_content)}")
        print(f"First 500 chars: {all_content[:500]}")

        # Try getting content from all paragraphs in main_content
        paragraphs = main_content.find_all('p')
        print(f"Found {len(paragraphs)} paragraphs in main content")
        for i, p in enumerate(paragraphs[:3]):
            p_text = p.get_text().strip()
            print(f"  P {i+1} ({len(p_text)} chars): {p_text[:100]}...")

    # Also try getting content from all content elements as a fallback
    content_elements = soup.find_all(['p', 'div', 'section', 'article', 'main', 'span'])
    all_content_parts = []
    for elem in content_elements:
        elem_text = elem.get_text(separator=' ', strip=True)
        if len(elem_text) > 10:  # Include any content over 10 chars
            all_content_parts.append(elem_text)

    if all_content_parts:
        all_content = ' '.join(all_content_parts)
        print(f"Content from all content elements: {len(all_content)} chars")
        print(f"First 500 chars: {all_content[:500]}")
    else:
        all_content = soup.get_text(separator=' ', strip=True)
        print(f"Full page content: {len(all_content)} chars")
        print(f"First 500 chars: {all_content[:500]}")

    # Apply the same cleaning as our code
    lines = [line.strip() for line in all_content.splitlines() if line.strip()]
    clean_text = ' '.join(line for line in lines if line and len(line) > 5)  # Only keep lines with more than 5 chars
    print(f"After cleaning (lines > 5 chars): {len(clean_text)} chars")
    print(f"First 500 chars: {clean_text[:500]}")

finally:
    driver.quit()