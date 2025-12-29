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

# Find the actual content structure
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

    print("=== FINDING ACTUAL CONTENT STRUCTURE ===")

    # Let's look for all divs that might contain the content
    all_divs = soup.find_all('div')

    # Look for divs with substantial text content that are NOT navigation elements
    content_divs = []
    for div in all_divs:
        text_content = div.get_text().strip()
        classes = div.get('class', [])
        id_attr = div.get('id', '')

        # Skip if it's a navigation element
        if any(nav_class in ''.join(classes).lower() for nav_class in ['nav', 'menu', 'sidebar', 'header', 'footer']):
            continue

        if len(text_content) > 100:  # Substantial content
            content_divs.append((len(text_content), div.name, classes, id_attr, text_content[:100]))

    # Sort by content length
    content_divs.sort(key=lambda x: x[0], reverse=True)

    print(f"Found {len(content_divs)} divs with substantial content (>100 chars):")
    for i, (length, tag, classes, id_attr, preview) in enumerate(content_divs[:10]):
        print(f"{i+1}. Length: {length}, Tag: {tag}, Classes: {classes}, ID: {id_attr}")
        print(f"   Preview: {preview}...")
        print()

    # Look for elements that might be the main content area
    print("=== CHECKING FOR SPECIFIC CONTENT PATTERNS ===")

    # Look for elements with class names that might indicate content
    content_indicators = [
        'doc', 'content', 'main', 'article', 'markdown', 'theme', 'container',
        'prose', 'content', 'body', 'text', 'document'
    ]

    for indicator in content_indicators:
        elements = soup.find_all(class_=re.compile(indicator, re.I))
        if elements:
            print(f"Elements with '{indicator}' in class:")
            for i, elem in enumerate(elements[:5]):  # Show first 5
                text = elem.get_text().strip()
                if len(text) > 50:  # Only show if substantial content
                    print(f"  {i+1}. Length: {len(text)}, Tag: {elem.name}, Classes: {elem.get('class', [])}")
                    print(f"     Preview: {text[:100]}...")
                    print()

    # Let's also try to find elements by their position in the DOM
    print("=== CHECKING MAIN ARTICLE STRUCTURE ===")

    # Look for the main content container - usually inside main or article tags
    main_tags = soup.find_all(['main', 'article', 'section'])
    for main_tag in main_tags:
        text_content = main_tag.get_text().strip()
        if len(text_content) > 50:
            print(f"Main tag found: {main_tag.name}")
            print(f"  Classes: {main_tag.get('class', [])}")
            print(f"  ID: {main_tag.get('id', '')}")
            print(f"  Content length: {len(text_content)}")
            print(f"  Preview: {text_content[:200]}...")
            print()

    # Let's also check for any element that contains the actual book content
    print("=== LOOKING FOR BOOK-SPECIFIC CONTENT ===")

    # Search for elements containing key phrases from the book
    book_keywords = ['Physical AI', 'Humanoid Robotics', 'ROS 2', 'NVIDIA Isaac', 'Vision-Language-Action', 'Digital Twin']

    for keyword in book_keywords:
        elements = soup.find_all(string=re.compile(re.escape(keyword), re.I))
        if elements:
            print(f"Elements containing '{keyword}':")
            for i, element in enumerate(elements[:3]):  # Show first 3
                parent = element.parent
                text = parent.get_text().strip()
                print(f"  {i+1}. Parent tag: {parent.name}, Classes: {parent.get('class', [])}")
                print(f"     Content: {text[:200]}...")
                print()

finally:
    driver.quit()