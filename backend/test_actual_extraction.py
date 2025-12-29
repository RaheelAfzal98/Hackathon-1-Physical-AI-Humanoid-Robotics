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

# Test the exact extraction process that our code uses
url = "https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app/docs/intro"

print("Testing exact extraction process for:", url)

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

    # Remove script and style elements (same as our code)
    for script in soup(["script", "style"]):
        script.decompose()

    # Remove navigation and UI elements first (same as our code)
    for element in soup(['nav', 'header', 'footer', 'aside']):
        element.decompose()
    for element in soup.find_all(class_=re.compile(r'nav|menu|sidebar|footer|button|btn|navbar|breadcrumbs|toc|footer')):
        element.decompose()
    for element in soup.find_all(class_=re.compile(r'sidebar|nav|menu|footer|button|btn|navbar|toc')):
        element.decompose()

    print("After removing navigation elements, HTML length:", len(str(soup)))

    # Extract main content using the same logic as our code
    main_content = (
        soup.find('div', class_=lambda x: x and 'theme-doc-markdown' in x) or  # Main content area in Docusaurus
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
        print(f"Main content length: {len(str(main_content))}")

        # Get all text content (same as our code)
        all_content = main_content.get_text(separator=' ', strip=True)
        print(f"Text content length from main_content: {len(all_content)}")

        # Apply same cleaning as our code
        lines = [line.strip() for line in all_content.splitlines() if line.strip()]
        clean_text = ' '.join(line for line in lines if line and len(line) > 5)  # Only keep lines with more than 5 chars

        print(f"Final clean text length: {len(clean_text)}")
        print(f"First 500 chars of clean text: {clean_text[:500]}")

        # Let's also check if the theme-doc-markdown div actually exists and what it contains
        theme_markdown_div = soup.find('div', class_=lambda x: x and 'theme-doc-markdown' in x)
        if theme_markdown_div:
            print(f"\nFound theme-doc-markdown div!")
            print(f"Tag: {theme_markdown_div.name}")
            print(f"Classes: {theme_markdown_div.get('class', [])}")
            print(f"Length: {len(str(theme_markdown_div))}")
            theme_content = theme_markdown_div.get_text(separator=' ', strip=True)
            print(f"Text content: {len(theme_content)} chars")
            print(f"First 500 chars: {theme_content[:500]}")
        else:
            print("\ntheme-doc-markdown div NOT found!")
            # Let's look for other possibilities
            all_divs = soup.find_all('div', class_=True)
            for div in all_divs:
                classes = div.get('class', [])
                if any('markdown' in c.lower() for c in classes):
                    print(f"Found div with markdown class: {classes}")
                    print(f"Content: {len(div.get_text())} chars")
                    print(f"Preview: {div.get_text()[:200]}...")

        # Let's also check the structure around the main content
        print(f"\nChecking parent structure...")
        if main_content.parent:
            print(f"Parent tag: {main_content.parent.name}")
            print(f"Parent classes: {main_content.parent.get('class', [])}")
            parent_content = main_content.parent.get_text(separator=' ', strip=True)
            print(f"Parent content length: {len(parent_content)}")

finally:
    driver.quit()