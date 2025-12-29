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

# Find content containers in the page
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

    print("=== ANALYZING HTML STRUCTURE ===")
    print(f"Total HTML length: {len(html_content)}")

    # Look for elements with substantial text content
    print("\n=== LOOKING FOR CONTENT-RICH ELEMENTS ===")

    # Find all elements and check their text content
    all_elements = soup.find_all()
    content_elements = []

    for element in all_elements[:100]:  # Check first 100 elements
        text_content = element.get_text().strip()
        if len(text_content) > 50:  # Elements with more than 50 characters of text
            tag_name = element.name
            classes = element.get('class', [])
            id_attr = element.get('id', '')
            content_elements.append((len(text_content), tag_name, classes, id_attr, text_content[:100]))

    # Sort by content length (descending)
    content_elements.sort(key=lambda x: x[0], reverse=True)

    print(f"Found {len(content_elements)} elements with substantial content (>50 chars):")
    for i, (length, tag, classes, id_attr, preview) in enumerate(content_elements[:10]):
        print(f"{i+1}. Length: {length}, Tag: {tag}, Classes: {classes}, ID: {id_attr}")
        print(f"   Preview: {preview}...")
        print()

    # Look for common Docusaurus content containers
    print("=== LOOKING FOR COMMON DOCUSAURUS CONTAINERS ===")
    docusaurus_selectors = [
        ('div', {'class': re.compile(r'docItem|theme|markdown|content', re.I)}),
        ('article', {'class': re.compile(r'doc|theme|content', re.I)}),
        ('main', {}),
        ('div', {'role': 'main'}),
        ('div', {'class': re.compile(r'main|content|doc|theme', re.I)}),
    ]

    for tag, attrs in docusaurus_selectors:
        elements = soup.find_all(tag, attrs)
        if elements:
            print(f"Found {len(elements)} {tag} elements with {attrs}:")
            for i, elem in enumerate(elements[:3]):
                text = elem.get_text().strip()
                print(f"  {i+1}. Length: {len(text)}, Classes: {elem.get('class', [])}, ID: {elem.get('id', '')}")
                print(f"     Preview: {text[:100]}...")
                print()

    # Look for content within specific common Docusaurus class patterns
    print("=== LOOKING FOR DOCUSAURUS-SPECIFIC PATTERNS ===")
    common_patterns = [
        r'theme-doc-markdown',
        r'docItem',
        r'docs',
        r'markdown',
        r'container',
        r'article',
        r'theme',
        r'content'
    ]

    for pattern in common_patterns:
        elements = soup.find_all(class_=re.compile(pattern, re.I))
        if elements:
            print(f"Elements matching pattern '{pattern}':")
            for i, elem in enumerate(elements[:3]):
                text = elem.get_text().strip()
                print(f"  {i+1}. Length: {len(text)}, Tag: {elem.name}, Classes: {elem.get('class', [])}")
                print(f"     Preview: {text[:100]}...")
                print()

finally:
    driver.quit()