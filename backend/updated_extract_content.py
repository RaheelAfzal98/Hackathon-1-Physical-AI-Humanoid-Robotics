"""
Updated extract_content method that uses Selenium for JavaScript rendering
"""
def extract_content(self, url: str) -> Optional[Dict[str, Any]]:
    """Extract clean text content from a given URL."""
    try:
        # Use Selenium to render JavaScript content
        chrome_options = Options()
        chrome_options.add_argument("--headless")  # Run in headless mode
        chrome_options.add_argument("--no-sandbox")
        chrome_options.add_argument("--disable-dev-shm-usage")
        chrome_options.add_argument("--disable-gpu")
        chrome_options.add_argument("--window-size=1920,1080")
        chrome_options.add_argument("--user-agent=Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36")

        service = Service(ChromeDriverManager().install())
        driver = webdriver.Chrome(service=service, options=chrome_options)

        try:
            driver.get(url)
            # Wait for the page to load and content to be rendered
            WebDriverWait(driver, 10).until(
                EC.presence_of_element_located((By.TAG_NAME, "body"))
            )

            # Wait a bit more for dynamic content to load
            time.sleep(3)

            # Get the page source after JavaScript execution
            html_content = driver.page_source
            title_text = driver.title

            # Parse with BeautifulSoup
            soup = BeautifulSoup(html_content, 'html.parser')

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

            # Extract main content - prioritize content in main containers
            # For Docusaurus sites, look for specific content containers
            main_content = (
                soup.find('article', class_=lambda x: x and 'theme-doc-markdown' in x) or
                soup.find('div', class_=lambda x: x and 'container' in x) or
                soup.find('main') or
                soup.find('article') or
                soup.find('div', class_=lambda x: x and ('content' in x.lower() or 'docItem' in x or 'markdown' in x)) or
                soup
            )

            # Get text content from the main content area
            if main_content:
                # Get all text content
                all_content = main_content.get_text(separator=' ', strip=True)

                # Split into lines and filter for actual content
                lines = [line.strip() for line in all_content.splitlines() if line.strip()]

                # Filter out lines that appear to be just titles/headers based on length and content
                content_parts = []
                for line in lines:
                    # Skip if it looks like a header/title (too short, ends with common header patterns)
                    if len(line) > 30:  # More substantial content
                        content_parts.append(line)
                    elif len(line) > 15 and not any(common in line.lower() for common in ['chapter', 'module', 'introduction', 'curriculum', '& humanoid robotics', 'physical ai & humanoid robotics']):
                        # Include if it's not obviously a title/module name
                        content_parts.append(line)

                if content_parts:
                    clean_text = ' '.join(content_parts)
                    text = clean_text
                else:
                    # If filtering removed everything, use original text
                    text = all_content
            else:
                # If no main content found, use the entire soup
                text = soup.get_text(separator=' ', strip=True)

            # Clean up text - remove extra whitespace
            lines = [line.strip() for line in text.splitlines() if line.strip()]
            clean_text = ' '.join(line for line in lines if line)

            # Log what we're extracting to help with debugging
            self.logger.info(f"Extracted content from {url}: {len(clean_text)} characters, title: {title_text[:50]}...")

            return {
                'url': url,
                'title': title_text,
                'content': clean_text,
                'timestamp': time.time()
            }
        finally:
            driver.quit()

    except Exception as e:
        self.logger.error(f"Error extracting content from {url}: {str(e)}")
        # Fallback to requests if Selenium fails
        try:
            headers = {
                'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
            }
            response = requests.get(url, headers=headers, timeout=10)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')

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

            # Extract main content - prioritize content in main containers
            # For Docusaurus sites, look for specific content containers
            main_content = (
                soup.find('article', class_=lambda x: x and 'theme-doc-markdown' in x) or
                soup.find('div', class_=lambda x: x and 'container' in x) or
                soup.find('main') or
                soup.find('article') or
                soup.find('div', class_=lambda x: x and ('content' in x.lower() or 'docItem' in x or 'markdown' in x)) or
                soup
            )

            # Get text content from the main content area
            if main_content:
                # Get all text content
                all_content = main_content.get_text(separator=' ', strip=True)

                # Split into lines and filter for actual content
                lines = [line.strip() for line in all_content.splitlines() if line.strip()]

                # Filter out lines that appear to be just titles/headers based on length and content
                content_parts = []
                for line in lines:
                    # Skip if it looks like a header/title (too short, ends with common header patterns)
                    if len(line) > 30:  # More substantial content
                        content_parts.append(line)
                    elif len(line) > 15 and not any(common in line.lower() for common in ['chapter', 'module', 'introduction', 'curriculum', '& humanoid robotics', 'physical ai & humanoid robotics']):
                        # Include if it's not obviously a title/module name
                        content_parts.append(line)

                if content_parts:
                    clean_text = ' '.join(content_parts)
                    text = clean_text
                else:
                    # If filtering removed everything, use original text
                    text = all_content
            else:
                # If no main content found, use the entire soup
                text = soup.get_text(separator=' ', strip=True)

            # Clean up text - remove extra whitespace
            lines = [line.strip() for line in text.splitlines() if line.strip()]
            clean_text = ' '.join(line for line in lines if line)

            # Extract title
            title = soup.find('title')
            title_text = title.get_text().strip() if title else ""

            # Log what we're extracting to help with debugging
            self.logger.info(f"Extracted content from {url}: {len(clean_text)} characters, title: {title_text[:50]}...")

            return {
                'url': url,
                'title': title_text,
                'content': clean_text,
                'timestamp': time.time()
            }
        except Exception as fallback_error:
            self.logger.error(f"Fallback extraction also failed for {url}: {str(fallback_error)}")
            return None