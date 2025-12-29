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

# Find the actual content structure - simplified version
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

    print("=== FINDING CONTENT DIVS ===")

    # Look for the specific theme-doc-markdown div
    theme_markdown_div = soup.find('div', class_='theme-doc-markdown')
    if theme_markdown_div:
        print(f"Found theme-doc-markdown div!")
        print(f"Classes: {theme_markdown_div.get('class', [])}")
        print(f"ID: {theme_markdown_div.get('id', '')}")
        content = theme_markdown_div.get_text(separator=' ', strip=True)
        print(f"Content length: {len(content)}")
        print(f"First 500 chars: {content[:500]}...")
    else:
        print("theme-doc-markdown div NOT found")
        # Try to find divs with both classes
        elements = soup.find_all('div', class_=re.compile(r'theme-doc-markdown'))
        if elements:
            print(f"Found {len(elements)} elements with 'theme-doc-markdown' in class:")
            for i, elem in enumerate(elements):
                content = elem.get_text(separator=' ', strip=True)
                print(f"  {i+1}. Length: {len(content)}, Classes: {elem.get('class', [])}")
                print(f"      Preview: {content[:100]}...")
        else:
            print("No elements found with 'theme-doc-markdown' in class")

    # Let's also look for the main content area structure
    print("\n=== MAIN CONTENT STRUCTURE ===")
    main_div = soup.find('div', id=re.compile(r'docusaurus_skipToContent'))
    if main_div:
        print(f"Found main content div with ID containing 'docusaurus_skipToContent'")
        print(f"Classes: {main_div.get('class', [])}")
        content = main_div.get_text(separator=' ', strip=True)
        print(f"Content length: {len(content)}")
        print(f"First 500 chars: {content[:500]}...")

    # Let's also check for the docRoot container
    doc_root = soup.find('div', class_='docRoot_UBD9')
    if doc_root:
        print(f"\nFound docRoot_UBD9 div!")
        content = doc_root.get_text(separator=' ', strip=True)
        print(f"Content length: {len(content)}")
        print(f"First 500 chars: {content[:500]}...")

finally:
    driver.quit()