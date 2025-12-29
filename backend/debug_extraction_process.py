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
import logging

# Debug the exact extraction process step by step
url = "https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app/docs/intro"

print("Debugging extraction process for:", url)

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

    # Step 1: Remove script and style elements
    for script in soup(["script", "style"]):
        script.decompose()
    print(f"After removing scripts and styles: {len(str(soup))}")

    # Step 2: Remove navigation and UI elements
    for element in soup(['nav', 'header', 'footer', 'aside']):
        element.decompose()
    print(f"After removing nav, header, footer, aside: {len(str(soup))}")

    for element in soup.find_all(class_=re.compile(r'nav|menu|sidebar|footer|button|btn|navbar|breadcrumbs|toc|footer')):
        element.decompose()
    print(f"After removing nav-related classes: {len(str(soup))}")

    for element in soup.find_all(class_=re.compile(r'sidebar|nav|menu|footer|button|btn|navbar|toc')):
        element.decompose()
    print(f"After removing more UI elements: {len(str(soup))}")

    # Step 3: Find main content using our exact logic
    main_content = (
        soup.find('div', class_='theme-doc-markdown') or  # Main content area in Docusaurus - highest priority
        soup.find('div', class_=lambda x: x and 'theme-doc-markdown' in x) or
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

    print(f"Main content found: {main_content is not None}")
    if main_content:
        print(f"Main content tag: {main_content.name}")
        print(f"Main content classes: {main_content.get('class', [])}")
        print(f"Main content HTML length: {len(str(main_content))}")

        # Step 4: Extract text content
        all_content = main_content.get_text(separator=' ', strip=True)
        print(f"All content text length: {len(all_content)}")
        print(f"First 100 chars: '{all_content[:100]}'")
        print(f"Last 100 chars: '{all_content[-100:] if len(all_content) > 100 else all_content}'")

        # Step 5: Split into lines
        lines = [line.strip() for line in all_content.splitlines() if line.strip()]
        print(f"Number of lines after splitting: {len(lines)}")
        if lines:
            print(f"First few lines: {lines[:3]}")

        # Step 6: Filter lines (our actual cleaning process)
        clean_lines = [line for line in lines if line and len(line) > 5]  # Only keep lines with more than 5 chars
        print(f"Number of lines after filtering (len > 5): {len(clean_lines)}")
        if clean_lines:
            print(f"First few clean lines: {clean_lines[:3]}")

        # Step 7: Join final content
        clean_text = ' '.join(clean_lines)
        print(f"Final clean text length: {len(clean_text)}")
        print(f"Final clean text preview: {clean_text[:200]}...")

        # Let's also check if there are many zero-width or special characters
        special_chars = [c for c in all_content if ord(c) < 32 or ord(c) == 8203]  # zero-width space and other control chars
        print(f"Special characters found: {len(special_chars)}")
        if special_chars:
            print(f"First few special chars: {special_chars[:10]}")

        # Check for zero-width spaces specifically
        zwsp_count = all_content.count('\u200b')  # zero-width space
        print(f"Zero-width spaces found: {zwsp_count}")

        # Try cleaning zero-width spaces
        cleaned_content = all_content.replace('\u200b', '').replace('\u200c', '').replace('\u200d', '')
        print(f"Content length after removing zero-width spaces: {len(cleaned_content)}")

        # Re-apply our filtering to cleaned content
        clean_lines_cleaned = [line for line in cleaned_content.splitlines() if line.strip() and len(line.strip()) > 5]
        final_clean_text = ' '.join([line.strip() for line in clean_lines_cleaned])
        print(f"Final text after cleaning special chars: {len(final_clean_text)}")
        print(f"Final text preview: {final_clean_text[:200]}...")

finally:
    driver.quit()