"""
Improved Web Content Crawler and Embedding Generator for RAG System

This script:
1. Crawls specified website URLs more effectively for Docusaurus sites
2. Extracts clean text content with better selectors
3. Chunks the content appropriately
4. Generates embeddings using Cohere
5. Stores the embeddings in Qdrant vector database
"""
import asyncio
import logging
from typing import List, Dict, Any, Optional
from urllib.parse import urljoin, urlparse
import requests
import re
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct, VectorParams, Distance
from concurrent.futures import ThreadPoolExecutor
import time
import hashlib
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from selenium.common.exceptions import TimeoutException
from webdriver_manager.chrome import ChromeDriverManager
from selenium.webdriver.chrome.service import Service


class ImprovedWebCrawler:
    def __init__(self, urls: List[str], max_depth: int = 2, max_pages: int = 100):
        self.urls = urls
        self.max_depth = max_depth
        self.max_pages = max_pages
        self.visited_urls = set()
        self.content_cache = {}

        # Set up logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

    def extract_content(self, url: str) -> Optional[Dict[str, Any]]:
        """Extract clean text content from a given URL with improved selectors."""
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
                main_content = None

                # Look for the specific theme-doc-markdown div which contains the main content
                theme_markdown_divs = soup.find_all('div', class_=re.compile(r'theme-doc-markdown'))
                if theme_markdown_divs:
                    # Combine all theme-doc-markdown divs as content might be split
                    combined_content = []
                    for div in theme_markdown_divs:
                        combined_content.append(str(div))
                    main_content = BeautifulSoup(''.join(combined_content), 'html.parser')
                else:
                    # Look for other common Docusaurus content containers
                    main_content = (
                        soup.find('article', class_=re.compile(r'theme-.*-doc-item--content')) or
                        soup.find('div', class_=re.compile(r'docItem-container')) or
                        soup.find('main', class_=re.compile(r'docs-page')) or
                        soup.find('div', class_=re.compile(r'theme-doc-content')) or
                        soup.find('div', class_=re.compile(r'doc-wrapper')) or
                        soup.find('div', class_=re.compile(r'main-wrapper')) or
                        soup.find('div', {'role': 'main'}) or  # Common role for main content
                        soup.find('main') or
                        soup.find('article') or
                        soup
                    )

                # Get text content from the main content area
                if main_content:
                    # Get all text content - be more inclusive now
                    all_content = main_content.get_text(separator=' ', strip=True)
                else:
                    # If no main content found, get content from paragraphs and content elements
                    content_elements = soup.find_all(['p', 'div', 'section', 'article', 'main', 'span', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'li', 'td', 'th'])
                    all_content_parts = []
                    for elem in content_elements:
                        elem_text = elem.get_text(separator=' ', strip=True)
                        if len(elem_text) > 10:  # Include any content over 10 chars
                            all_content_parts.append(elem_text)

                    if all_content_parts:
                        all_content = ' '.join(all_content_parts)
                    else:
                        all_content = soup.get_text(separator=' ', strip=True)

                # Clean up text - remove extra whitespace and clean up
                lines = [line.strip() for line in all_content.splitlines() if line.strip()]
                clean_text = ' '.join(line for line in lines if line and len(line) > 5)  # Only keep lines with more than 5 chars

                # Additional cleaning for common issues
                # Remove duplicate spaces
                clean_text = re.sub(r'\s+', ' ', clean_text)

                # Remove common navigation patterns that might have slipped through
                clean_text = re.sub(r'«\s.*?\s»', '', clean_text)  # Remove navigation arrows
                clean_text = re.sub(r'\s+', ' ', clean_text).strip()

                # Log what we're extracting to help with debugging
                self.logger.info(f"Extracted content from {url}: {len(clean_text)} characters, title: {title_text[:100]}...")

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
                main_content = None

                # Look for the specific theme-doc-markdown div which contains the main content
                theme_markdown_divs = soup.find_all('div', class_=re.compile(r'theme-doc-markdown'))
                if theme_markdown_divs:
                    # Combine all theme-doc-markdown divs as content might be split
                    combined_content = []
                    for div in theme_markdown_divs:
                        combined_content.append(str(div))
                    main_content = BeautifulSoup(''.join(combined_content), 'html.parser')
                else:
                    # Look for other common Docusaurus content containers
                    main_content = (
                        soup.find('article', class_=re.compile(r'theme-.*-doc-item--content')) or
                        soup.find('div', class_=re.compile(r'docItem-container')) or
                        soup.find('main', class_=re.compile(r'docs-page')) or
                        soup.find('div', class_=re.compile(r'theme-doc-content')) or
                        soup.find('div', class_=re.compile(r'doc-wrapper')) or
                        soup.find('div', class_=re.compile(r'main-wrapper')) or
                        soup.find('div', {'role': 'main'}) or  # Common role for main content
                        soup.find('main') or
                        soup.find('article') or
                        soup
                    )

                # Get text content from the main content area
                if main_content:
                    # Get all text content - be more inclusive now
                    all_content = main_content.get_text(separator=' ', strip=True)
                else:
                    # If no main content found, get content from paragraphs and content elements
                    content_elements = soup.find_all(['p', 'div', 'section', 'article', 'main', 'span', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'li', 'td', 'th'])
                    all_content_parts = []
                    for elem in content_elements:
                        elem_text = elem.get_text(separator=' ', strip=True)
                        if len(elem_text) > 10:  # Include any content over 10 chars
                            all_content_parts.append(elem_text)

                    if all_content_parts:
                        all_content = ' '.join(all_content_parts)
                    else:
                        all_content = soup.get_text(separator=' ', strip=True)

                # Clean up text - remove extra whitespace and clean up
                lines = [line.strip() for line in all_content.splitlines() if line.strip()]
                clean_text = ' '.join(line for line in lines if line and len(line) > 5)  # Only keep lines with more than 5 chars

                # Additional cleaning for common issues
                # Remove duplicate spaces
                clean_text = re.sub(r'\s+', ' ', clean_text)

                # Remove common navigation patterns that might have slipped through
                clean_text = re.sub(r'«\s.*?\s»', '', clean_text)  # Remove navigation arrows
                clean_text = re.sub(r'\s+', ' ', clean_text).strip()

                # Extract title
                title = soup.find('title')
                title_text = title.get_text().strip() if title else ""

                # Log what we're extracting to help with debugging
                self.logger.info(f"Extracted content from {url}: {len(clean_text)} characters, title: {title_text[:100]}...")

                return {
                    'url': url,
                    'title': title_text,
                    'content': clean_text,
                    'timestamp': time.time()
                }
            except Exception as fallback_error:
                self.logger.error(f"Fallback extraction also failed for {url}: {str(fallback_error)}")
                return None

    def get_all_urls(self) -> List[str]:
        """Get all URLs from the seed URLs including linked pages up to max_depth."""
        all_urls = set(self.urls)
        current_depth = 0

        while current_depth < self.max_depth and len(all_urls) < self.max_pages:
            current_urls = list(all_urls - self.visited_urls)
            if not current_urls:
                break

            # Limit the number of new pages to process
            current_urls = current_urls[:min(50, self.max_pages - len(all_urls))]

            new_urls = set()
            for url in current_urls:
                if len(all_urls) >= self.max_pages:
                    break
                self.visited_urls.add(url)

                try:
                    headers = {
                        'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36'
                    }
                    response = requests.get(url, headers=headers, timeout=10)
                    soup = BeautifulSoup(response.content, 'html.parser')

                    # Find all links
                    for link in soup.find_all('a', href=True):
                        href = link['href']
                        full_url = urljoin(url, href)

                        # Only add URLs from the same domain and with /docs/ path (for textbook content)
                        if (urlparse(url).netloc == urlparse(full_url).netloc and
                            ('/docs/' in full_url or '/module-' in full_url or '/chapter-' in full_url)):
                            if full_url not in self.visited_urls:
                                new_urls.add(full_url)
                except Exception as e:
                    self.logger.error(f"Error crawling {url}: {str(e)}")

            all_urls.update(new_urls)
            current_depth += 1

        return list(all_urls)[:self.max_pages]


class ContentChunker:
    def __init__(self, chunk_size: int = 512, overlap: int = 50):
        self.chunk_size = chunk_size
        self.overlap = overlap

    def chunk_text(self, text: str, source_url: str, title: str) -> List[Dict[str, Any]]:
        """Chunk text into smaller pieces with overlap."""
        if len(text) <= self.chunk_size:
            return [{
                'content': text,
                'metadata': {
                    'source_url': source_url,
                    'page_title': title,
                    'chunk_index': 0,
                    'document_id': hashlib.md5((source_url + text[:100]).encode()).hexdigest()
                }
            }]

        chunks = []
        start_idx = 0
        chunk_index = 0

        while start_idx < len(text):
            end_idx = start_idx + self.chunk_size

            # If this isn't the last chunk, try to break at sentence boundary
            if end_idx < len(text):
                # Look for sentence boundaries near the end
                snippet = text[start_idx:end_idx]
                last_period = snippet.rfind('.')
                last_exclamation = snippet.rfind('!')
                last_question = snippet.rfind('?')
                last_space = snippet.rfind(' ')

                # Use the closest boundary to the end that makes sense
                boundary = max(last_period, last_exclamation, last_question, last_space)
                if boundary > len(snippet) // 2:  # Only if it's not too early
                    end_idx = start_idx + boundary + 1

            chunk_text = text[start_idx:end_idx].strip()

            if chunk_text:  # Only add non-empty chunks with substantial content
                if len(chunk_text) > 50:  # Only include chunks with meaningful content
                    chunks.append({
                        'content': chunk_text,
                        'metadata': {
                            'source_url': source_url,
                            'page_title': title,
                            'chunk_index': chunk_index,
                            'document_id': hashlib.md5((source_url + chunk_text[:100]).encode()).hexdigest()
                        }
                    })
                    chunk_index += 1

            # Move to next chunk with overlap
            start_idx = end_idx - self.overlap if end_idx < len(text) else len(text)

            # If we're past the end, break
            if start_idx >= len(text):
                break

        return chunks


class EmbeddingGenerator:
    def __init__(self, cohere_api_key: str, model: str = "embed-english-v3.0"):
        self.cohere_client = cohere.Client(cohere_api_key)
        self.model = model

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a list of texts using Cohere."""
        try:
            response = self.cohere_client.embed(
                texts=texts,
                model=self.model,
                input_type="search_document"  # Using search_document for content chunks
            )
            return [embedding for embedding in response.embeddings]
        except Exception as e:
            logging.error(f"Error generating embeddings: {str(e)}")
            raise


class QdrantStorage:
    def __init__(self, url: str, api_key: str, collection_name: str):
        self.client = QdrantClient(
            url=url,
            api_key=api_key,
            # Enable HTTPS if needed
            https=True if "https" in url else False
        )
        self.collection_name = collection_name

    def init_collection(self, vector_size: int = 1024):  # Cohere embed-english-v3.0 returns 1024-dim vectors
        """Initialize the Qdrant collection if it doesn't exist."""
        try:
            # Check if collection exists
            self.client.get_collection(collection_name=self.collection_name)
            logging.info(f"Collection {self.collection_name} already exists")
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
            )
            logging.info(f"Created collection {self.collection_name}")

    def store_embeddings(self, chunks: List[Dict[str, Any]], embeddings: List[List[float]]):
        """Store chunks and embeddings in Qdrant."""
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            # Only store chunks with non-empty content
            if chunk['content'].strip() and len(chunk['content']) > 50:  # Only store meaningful content
                point_id = hashlib.md5((chunk['metadata']['document_id'] + str(chunk['metadata']['chunk_index'])).encode()).hexdigest()

                point = PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "content": chunk['content'],
                        "metadata": chunk['metadata']
                    }
                )
                points.append(point)

        if points:  # Only upload if there are points to store
            # Upload points to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logging.info(f"Stored {len(points)} embeddings in Qdrant collection {self.collection_name}")
        else:
            logging.warning("No points to store in Qdrant after filtering empty chunks")


class ImprovedIngestionPipeline:
    def __init__(self, cohere_api_key: str, qdrant_url: str, qdrant_api_key: str,
                 collection_name: str = "book_embeddings"):
        self.crawler = None
        self.chunker = ContentChunker()
        self.embedding_gen = EmbeddingGenerator(cohere_api_key)
        self.qdrant_storage = QdrantStorage(qdrant_url, qdrant_api_key, collection_name)

        # Set up logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

    def run_ingestion(self, urls: List[str], chunk_size: int = 512, overlap: int = 50):
        """Run the full ingestion pipeline."""
        self.logger.info("Starting improved ingestion pipeline...")

        # Initialize Qdrant collection
        self.qdrant_storage.init_collection()

        # Crawl the websites with deeper exploration
        self.logger.info(f"Crawling websites: {urls}")
        self.crawler = ImprovedWebCrawler(urls, max_depth=2)  # Increased depth to explore more pages
        all_urls = self.crawler.get_all_urls()
        self.logger.info(f"Found {len(all_urls)} URLs to process")

        # Extract content from all URLs
        contents = []
        for i, url in enumerate(all_urls):
            self.logger.info(f"Processing {i+1}/{len(all_urls)}: {url}")
            content = self.crawler.extract_content(url)
            if content and len(content['content']) > 50:  # Only include content with meaningful length
                contents.append(content)

        self.logger.info(f"Successfully extracted content from {len(contents)} pages")

        # Chunk the content
        self.logger.info("Chunking content...")
        all_chunks = []
        for content in contents:
            chunks = self.chunker.chunk_text(content['content'], content['url'], content['title'])
            all_chunks.extend(chunks)

        self.logger.info(f"Created {len(all_chunks)} content chunks")

        # Generate embeddings
        self.logger.info("Generating embeddings...")
        # Filter out empty content before embedding
        texts_to_embed = [chunk['content'] for chunk in all_chunks if chunk['content'].strip() and len(chunk['content']) > 50]
        self.logger.info(f"Found {len(texts_to_embed)} non-empty chunks to embed")

        if not texts_to_embed:
            self.logger.warning("No content to embed after filtering empty chunks")
            return  # Exit early if no content to process

        # Process in batches to avoid rate limits
        batch_size = 20  # Cohere has limits on batch size
        all_embeddings = []

        for i in range(0, len(texts_to_embed), batch_size):
            batch = texts_to_embed[i:i+batch_size]
            batch_embeddings = self.embedding_gen.generate_embeddings(batch)
            all_embeddings.extend(batch_embeddings)
            self.logger.info(f"Processed batch {i//batch_size + 1}/{(len(texts_to_embed)-1)//batch_size + 1}")

        # Store in Qdrant
        self.logger.info("Storing embeddings in Qdrant...")
        self.qdrant_storage.store_embeddings(all_chunks, all_embeddings)

        self.logger.info("Improved ingestion pipeline completed successfully!")


if __name__ == "__main__":
    import os
    from src.config.settings import settings

    # Get configuration from environment variables
    cohere_api_key = os.getenv("COHERE_API_KEY") or settings.cohere_api_key
    qdrant_url = os.getenv("QDRANT_URL") or settings.qdrant_url
    qdrant_api_key = os.getenv("QDRANT_API_KEY") or settings.qdrant_api_key
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", settings.qdrant_collection_name)

    # Define the URLs to crawl - target the docs section specifically
    base_url = 'https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app'
    urls_to_crawl = [
        f'{base_url}/docs/intro',
        f'{base_url}/docs/module-1-ros2/chapter-1-introduction',
        f'{base_url}/docs/module-2-digital-twin/chapter-1-introduction',
        f'{base_url}/docs/module-3-ai-robot-brain/chapter-1-introduction',
        f'{base_url}/docs/module-4-vla/chapter-1-introduction',
        # Add base URL to get additional content
        base_url
    ]

    print('Starting improved ingestion of AI book content into Qdrant...')
    print(f'Using Qdrant collection: {collection_name}')
    print(f'Crawling the following URLs: {urls_to_crawl}')

    try:
        # Create ingestion pipeline with your settings
        ingestion_pipeline = ImprovedIngestionPipeline(
            cohere_api_key=cohere_api_key,
            qdrant_url=qdrant_url,
            qdrant_api_key=qdrant_api_key,
            collection_name=collection_name
        )

        print('Starting ingestion process...')

        # Run the ingestion with more verbose output
        ingestion_pipeline.run_ingestion(
            urls=urls_to_crawl,
            chunk_size=512,  # Appropriate for technical content
            overlap=50
        )

        print('\nImproved ingestion completed successfully!')
        print(f'Your book content has been embedded and stored in Qdrant collection: {collection_name}')
        print('You can now use the retrieval API to search through your book content.')

    except Exception as e:
        print(f'Error during ingestion: {str(e)}')
        import traceback
        traceback.print_exc()