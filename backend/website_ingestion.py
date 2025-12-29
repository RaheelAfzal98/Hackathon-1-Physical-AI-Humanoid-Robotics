"""
AI Data Ingestion and Vectorization Agent

This script extracts text content from a website and stores it as vector embeddings in a Qdrant database using Cohere embeddings.

Website URL (content source):
https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app/sitemap.xml

Vectorization Model:
- Provider: Cohere
- Model: embed-english-v3.0
- Output Vector Size: 1024

Qdrant Configuration:
- Qdrant URL: "https://16d81df7-6903-4b40-8490-a7413250cf8d.europe-west3-0.gcp.cloud.qdrant.io:6333"
- Qdrant API Key: "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.fKvVQx8n78Z3HPK2FZPFq1c6kTbYc0QfeoQZN6IwTPc"
- Collection Name: "physical_ai_humanoid_docs"
- Distance Metric: Cosine
"""

import os
import sys
import re
import uuid
import requests
import xml.etree.ElementTree as ET
from urllib.parse import urljoin
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct, VectorParams, Distance
import tiktoken
import logging
from typing import List, Dict, Any, Optional

# Add the necessary paths to sys.path
sys.path.append('.')
sys.path.append('src')

# Load environment variables from .env file if it exists
if os.path.exists('.env'):
    with open('.env', 'r') as file:
        for line in file:
            if line.strip() and not line.startswith('#'):
                key, value = line.strip().split('=', 1)
                os.environ[key] = value

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class WebsiteCrawler:
    def __init__(self, sitemap_url: str, max_pages: int = 100):
        self.sitemap_url = sitemap_url
        self.max_pages = max_pages
        self.visited_urls = set()
        self.content_cache = {}
        
    def parse_sitemap(self) -> List[str]:
        """Parse the sitemap and extract all URLs"""
        try:
            response = requests.get(self.sitemap_url)
            response.raise_for_status()
            
            root = ET.fromstring(response.content)
            
            # Handle different namespace formats
            namespaces = {
                'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9',
                'default': 'http://www.sitemaps.org/schemas/sitemap/0.9'
            }
            
            urls = []
            # Try both namespace options
            for tag in ['{http://www.sitemaps.org/schemas/sitemap/0.9}loc', '{default}loc', 'loc']:
                loc_elements = root.findall(f".//{tag}")
                if loc_elements:
                    urls.extend([elem.text.strip() for elem in loc_elements])
                    break
        
            # Replace placeholder domain with actual domain if needed
            actual_domain = "https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app"
            corrected_urls = [
                url.replace("https://your-vercel-project.vercel.app", actual_domain)
                for url in urls
            ]
            
            # Remove duplicates while preserving order
            seen = set()
            unique_urls = []
            for url in corrected_urls:
                if url not in seen:
                    seen.add(url)
                    unique_urls.append(url)
                    
            logger.info(f"Parsed {len(unique_urls)} unique URLs from sitemap")
            return unique_urls[:self.max_pages]
        
        except Exception as e:
            logger.error(f"Error parsing sitemap: {str(e)}")
            return []

    def extract_meaningful_content(self, url: str) -> Optional[Dict[str, Any]]:
        """Extract all meaningful textual content from a URL with retry logic"""
        max_retries = 3
        for attempt in range(max_retries):
            try:
                headers = {
                    'User-Agent': ('Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 '
                                  '(KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36')
                }
                response = requests.get(url, headers=headers, timeout=15)
                response.raise_for_status()

                soup = BeautifulSoup(response.content, 'html.parser')

                # Remove navigation, footers, and other UI elements (but preserve main content)
                for element in soup(['nav', 'header', 'footer', 'aside']):
                    element.decompose()

                # Remove specific classes commonly used for navigation or UI elements
                for element in soup.find_all(class_=re.compile(r'nav|menu|sidebar|footer|button|btn|navbar|breadcrumbs|toc|footer')):
                    element.decompose()

                # First, try to find the main content area (Docusaurus specific)
                # Docusaurus typically puts content in various containers
                main_content = (
                    soup.find('article', class_=re.compile(r'theme-doc-markdown|markdown', re.I)) or
                    soup.find('main') or
                    soup.find('article') or
                    soup.find('div', class_=re.compile(r'docItem|content|main', re.I)) or
                    soup
                )

                # Extract various types of content
                content_parts = []

                # Extract headings (h1 to h6)
                for heading in soup.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6']):
                    heading_text = heading.get_text(strip=True)
                    if heading_text:
                        content_parts.append({
                            'text': heading_text,
                            'type': 'heading',
                            'level': heading.name
                        })

                # Extract paragraphs and other meaningful text content
                for element in soup.find_all(['p', 'li', 'td', 'th']):
                    element_text = element.get_text(strip=True)
                    if element_text and len(element_text) > 10:  # Only meaningful text
                        # Determine type based on tag
                        if element.name == 'li':
                            content_type = 'list_item'
                        elif element.name in ['td', 'th']:
                            content_type = 'table_content'
                        else:
                            content_type = 'paragraph'

                        content_parts.append({
                            'text': element_text,
                            'type': content_type,
                            'level': None
                        })

                # Extract section titles/descriptions
                for div in soup.find_all(['div', 'section'], class_=lambda x: x and (
                    'title' in x or 'section' in x or 'header' in x or 'description' in x or 'content' in x
                )):
                    div_text = div.get_text(strip=True)
                    if div_text and len(div_text) > 20:  # Avoid short text
                        content_parts.append({
                            'text': div_text,
                            'type': 'section_title' if 'title' in (div.get('class', []) or '') else 'description',
                            'level': None
                        })

                # Overall page title
                title_tag = soup.find('title')
                page_title = title_tag.get_text().strip() if title_tag else ""

                # Meta description
                meta_desc = soup.find('meta', attrs={'name': 'description'})
                meta_description = meta_desc.get('content', '').strip() if meta_desc else ""

                if content_parts:
                    # Combine all content together
                    all_text = ' '.join([part['text'] for part in content_parts])

                    # Clean up text
                    all_text = re.sub(r'\s+', ' ', all_text).strip()

                    # Add metadata
                    return {
                        'url': url,
                        'title': page_title,
                        'meta_description': meta_description,
                        'content_parts': content_parts,
                        'full_content': all_text,
                        'timestamp': response.headers.get('last-modified', '')
                    }
                else:
                    logger.warning(f"No content extracted from {url}")

                    # Let's try a different approach - look for content even if it's not in semantic tags
                    # Extract all text and see if there's valuable content
                    all_text = soup.get_text(separator=' ', strip=True)
                    lines = [line.strip() for line in all_text.splitlines() if line.strip()]
                    meaningful_text = ' '.join([line for line in lines if len(line) > 10 and not re.match(r'^[A-Z\s]+$', line)])

                    # Check if the meaningful text is mostly just titles/headers and not actual content
                    if meaningful_text and len(meaningful_text) > 50:
                        # Check if this is mostly repetitive or title-like content
                        # by looking for patterns that suggest it's just metadata
                        text_lower = meaningful_text.lower()
                        title_in_text = page_title.lower() in text_lower if page_title else False

                        # Count how many times the title appears in the content
                        if page_title and title_in_text:
                            title_count = text_lower.count(page_title.lower())
                            # Only reject if the title appears more than 3 times and makes up more than 50% of the content
                            title_occurrence_ratio = (len(page_title) * title_count) / len(meaningful_text)
                            if title_count > 3 and title_occurrence_ratio > 0.5:
                                logger.warning(f"Content for {url} appears to be mostly title repetition, skipping")
                                return None

                        content_parts = [{
                            'text': meaningful_text,
                            'type': 'paragraph',
                            'level': None
                        }]

                        return {
                            'url': url,
                            'title': page_title,
                            'meta_description': meta_description,
                            'content_parts': content_parts,
                            'full_content': meaningful_text,
                            'timestamp': response.headers.get('last-modified', '')
                        }

                    return None

            except requests.exceptions.RequestException as e:
                logger.warning(f"Attempt {attempt + 1} failed for {url}: {str(e)}")
                if attempt == max_retries - 1:  # Last attempt
                    logger.error(f"Failed to extract content from {url} after {max_retries} attempts: {str(e)}")
                    return None
                time.sleep(2 ** attempt)  # Exponential backoff
            except Exception as e:
                logger.error(f"Error extracting content from {url}: {str(e)}")
                return None


class TextCleaner:
    @staticmethod
    def clean_text(text: str) -> str:
        """Clean the extracted text by removing unwanted patterns"""
        if not text:
            return ""
        
        # Remove extra whitespace
        text = re.sub(r'\s+', ' ', text)
        
        # Remove special characters but keep basic punctuation
        text = re.sub(r'[^\w\s\n\r\t.,!?;:\'"()-]', ' ', text)
        
        # Normalize quotes and dashes
        text = re.sub(r'[“”]', '"', text)
        text = re.sub(r'[‘’]', "'", text)
        text = re.sub(r'[–—]', '-', text)
        
        # Normalize whitespace again after substitutions
        text = re.sub(r'\s+', ' ', text).strip()
        
        return text


class TextChunker:
    def __init__(self, min_tokens: int = 300, max_tokens: int = 500):
        self.min_tokens = min_tokens
        self.max_tokens = max_tokens
        self.tokenizer = tiktoken.encoding_for_model("gpt-3.5-turbo")  # Used for token estimation
    
    def count_tokens(self, text: str) -> int:
        """Estimate number of tokens in text"""
        return len(self.tokenizer.encode(text))
    
    def chunk_text(self, content_parts: List[Dict], source_url: str) -> List[Dict[str, Any]]:
        """Chunk text into 300-500 token segments preserving semantic boundaries"""
        chunks = []
        current_chunk = []
        current_token_count = 0
        chunk_index = 0  # Track chunk index

        for part in content_parts:
            part_text = part['text']
            part_tokens = self.count_tokens(part_text)

            # If adding this part would exceed max tokens, start a new chunk
            if current_chunk and (current_token_count + part_tokens > self.max_tokens):
                # Only create chunk if it has enough content
                if current_token_count >= self.min_tokens:
                    chunk_content = ' '.join([p['text'] for p in current_chunk])
                    chunk_type = self._determine_chunk_type(current_chunk)

                    chunks.append({
                        'content': chunk_content,
                        'source_url': source_url,
                        'section_title': self._extract_section_title(current_chunk),
                        'content_type': chunk_type,
                        'token_count': current_token_count,
                        'chunk_index': chunk_index
                    })

                    current_chunk = [part]
                    current_token_count = part_tokens
                    chunk_index += 1  # Increment chunk index for next chunk
                else:
                    # If current chunk is too small, append to it anyway and continue
                    current_chunk.append(part)
                    current_token_count += part_tokens
            else:
                # Add to current chunk
                current_chunk.append(part)
                current_token_count += part_tokens

                # If this chunk has reached the minimum size threshold, create it
                if current_token_count >= self.max_tokens:
                    chunk_content = ' '.join([p['text'] for p in current_chunk])
                    chunk_type = self._determine_chunk_type(current_chunk)

                    chunks.append({
                        'content': chunk_content,
                        'source_url': source_url,
                        'section_title': self._extract_section_title(current_chunk),
                        'content_type': chunk_type,
                        'token_count': current_token_count,
                        'chunk_index': chunk_index
                    })

                    current_chunk = []
                    current_token_count = 0
                    chunk_index += 1  # Increment chunk index for next chunk

        # Add remaining content if there's any
        if current_chunk:
            chunk_content = ' '.join([p['text'] for p in current_chunk])
            chunk_type = self._determine_chunk_type(current_chunk)

            chunks.append({
                'content': chunk_content,
                'source_url': source_url,
                'section_title': self._extract_section_title(current_chunk),
                'content_type': chunk_type,
                'token_count': current_token_count,
                'chunk_index': chunk_index
            })

        return chunks
    
    def _determine_chunk_type(self, content_parts: List[Dict]) -> str:
        """Determine the content type based on the chunk content"""
        first_part_type = content_parts[0]['type']
        
        if first_part_type == 'heading':
            text = content_parts[0]['text'].lower()
            if 'intro' in text or 'overview' in text or 'welcome' in text:
                return 'overview'
            elif 'hero' in text or 'introduction' in text:
                return 'hero'
            else:
                return 'section'
        elif first_part_type == 'section_title':
            return 'section'
        else:
            return 'content'
    
    def _extract_section_title(self, content_parts: List[Dict]) -> str:
        """Extract a representative title for the chunk"""
        # Look for headings first
        for part in content_parts:
            if part['type'] == 'heading':
                return part['text'][:100]  # Return first 100 chars of heading
        
        # If no heading, use first part of content
        if content_parts:
            first_part_text = content_parts[0]['text']
            return first_part_text[:100] if len(first_part_text) > 100 else first_part_text
            
        return "Untitled Section"


class EmbeddingGenerator:
    def __init__(self, cohere_api_key: str, model: str = "embed-english-v3.0"):
        self.cohere_client = cohere.Client(cohere_api_key)
        self.model = model  # Updated to the required model
        
    def generate_embeddings(self, texts: List[str], input_types: List[str] = None) -> List[List[float]]:
        """Generate embeddings for a list of texts using Cohere"""
        if input_types is None:
            input_types = ["search_document"] * len(texts)
        
        try:
            response = self.cohere_client.embed(
                texts=texts,
                model=self.model,
                input_type="search_document"  # Using search_document for content chunks
            )
            return [embedding for embedding in response.embeddings]
        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}")
            raise


class QdrantManager:
    def __init__(self, url: str, api_key: str, collection_name: str):
        # Match the working configuration from the original script
        self.client = QdrantClient(
            url=url,
            api_key=api_key,
            # Enable HTTPS if needed
            https=True if "https" in url else False
        )
        self.collection_name = collection_name
        self.url = url
        self.api_key = api_key

    def create_collection_if_not_exists(self, vector_size: int = 1024):
        """Create the Qdrant collection with cosine distance if it doesn't exist"""
        try:
            # Check if collection exists
            self.client.get_collection(collection_name=self.collection_name)
            logger.info(f"Collection {self.collection_name} already exists")
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
            )
            logger.info(f"Created collection {self.collection_name} with cosine distance metric")

    def store_embeddings(self, chunks: List[Dict[str, Any]], embeddings: List[List[float]]) -> int:
        """Store chunks and embeddings in Qdrant with retry logic"""
        points = []
        processed_count = 0

        for chunk, embedding in zip(chunks, embeddings):
            # Generate a unique ID for this chunk
            chunk_id = str(uuid.uuid4())

            point = PointStruct(
                id=chunk_id,
                vector=embedding,
                payload={
                    "source_url": chunk['source_url'],
                    "page_title": chunk.get('section_title', ''),  # Using section_title as page_title
                    "section_heading": chunk.get('section_title', ''),  # Using section_title as section_heading
                    "chunk_index": chunk.get('chunk_index', 0),  # Add chunk index if available
                    "content": chunk['content'],
                    "content_type": chunk.get('content_type', 'content'),
                    "token_count": chunk.get('token_count', 0)
                }
            )
            points.append(point)
            processed_count += 1

        # Upload points to Qdrant with retry logic
        if points:
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    self.client.upsert(
                        collection_name=self.collection_name,
                        points=points
                    )
                    logger.info(f"Successfully stored {len(points)} embeddings in Qdrant collection {self.collection_name}")
                    return processed_count
                except Exception as e:
                    logger.warning(f"Attempt {attempt + 1} failed to store embeddings in Qdrant: {str(e)}")
                    if attempt == max_retries - 1:  # Last attempt
                        logger.error(f"Failed to store embeddings in Qdrant after {max_retries} attempts: {str(e)}")
                        raise
                    time.sleep(2 ** attempt)  # Exponential backoff

        return processed_count


class DataIngestionAgent:
    def __init__(self, cohere_api_key: str, qdrant_url: str, qdrant_api_key: str, 
                 collection_name: str = "physical_ai_humanoid_docs"):
        self.crawler = None
        self.cleaner = TextCleaner()
        self.chunker = TextChunker(min_tokens=300, max_tokens=500)
        self.embedding_gen = EmbeddingGenerator(cohere_api_key, "embed-english-v3.0")
        self.qdrant_manager = QdrantManager(qdrant_url, qdrant_api_key, collection_name)
    
    def ingest_website(self, sitemap_url: str) -> Dict[str, Any]:
        """Main method to run the entire ingestion pipeline"""
        logger.info("Starting website ingestion pipeline...")
        
        # Initialize Qdrant collection
        self.qdrant_manager.create_collection_if_not_exists(vector_size=1024)
        
        # Crawl the website using sitemap
        logger.info(f"Crawling website using sitemap: {sitemap_url}")
        self.crawler = WebsiteCrawler(sitemap_url)
        urls = self.crawler.parse_sitemap()
        
        if not urls:
            logger.error("No URLs found in sitemap")
            return {
                "total_chunks_embedded": 0,
                "collection_created_or_updated": False,
                "skipped_content": [],
                "errors": ["No URLs found in sitemap"]
            }
        
        logger.info(f"Processing {len(urls)} URLs from sitemap")
        
        # Extract content from all URLs
        all_chunks = []
        skipped_content = []
        errors = []
        
        for i, url in enumerate(urls):
            logger.info(f"Processing {i+1}/{len(urls)}: {url}")
            
            content_data = self.crawler.extract_meaningful_content(url)
            if content_data:
                # Clean the content
                content_data['full_content'] = self.cleaner.clean_text(content_data['full_content'])
                
                # Chunk the content
                chunks = self.chunker.chunk_text(
                    content_data['content_parts'],
                    content_data['url']
                )
                
                # Add content type info to chunks
                for chunk in chunks:
                    # Check if this chunk is already processed (avoid duplicates)
                    chunk_text_hash = hash(chunk['content'])
                    if chunk_text_hash not in [hash(c['content']) for c in all_chunks]:
                        all_chunks.append(chunk)
                    else:
                        skipped_content.append({
                            "url": url,
                            "reason": "duplicate content"
                        })
                
                logger.info(f"  - Created {len(chunks)} chunks from {url}")
            else:
                skipped_content.append({
                    "url": url,
                    "reason": "no content extracted"
                })
        
        logger.info(f"Total chunks prepared for embedding: {len(all_chunks)}")
        
        if not all_chunks:
            logger.error("No content to embed")
            return {
                "total_chunks_embedded": 0,
                "collection_created_or_updated": True,  # Collection was still created/verified
                "skipped_content": skipped_content,
                "errors": ["No content extracted from any URLs"] + errors
            }
        
        # Generate embeddings
        logger.info("Generating embeddings...")
        texts_to_embed = [chunk['content'] for chunk in all_chunks]
        
        # Process in smaller batches to handle potential rate limits
        batch_size = 50  # Adjust based on Cohere limits
        all_embeddings = []
        
        for i in range(0, len(texts_to_embed), batch_size):
            batch = texts_to_embed[i:i+batch_size]
            try:
                batch_embeddings = self.embedding_gen.generate_embeddings(batch)
                all_embeddings.extend(batch_embeddings)
                logger.info(f"  - Processed batch {i//batch_size + 1}/{(len(texts_to_embed)-1)//batch_size + 1}")
            except Exception as e:
                logger.error(f"Error processing batch {i//batch_size + 1}: {str(e)}")
                errors.append(f"Batch {i//batch_size + 1} failed: {str(e)}")
                # Add empty embeddings for failed items to maintain alignment
                all_embeddings.extend([[]] * len(batch))
        
        # Remove any chunks that had failed embeddings
        valid_data = [(chunk, emb) for chunk, emb in zip(all_chunks, all_embeddings) if emb]
        if len(valid_data) != len(all_chunks):
            logger.warning(f"Removed {len(all_chunks) - len(valid_data)} chunks due to embedding errors")
        
        valid_chunks, valid_embeddings = zip(*valid_data) if valid_data else ([], [])
        
        # Store in Qdrant
        if valid_embeddings:
            logger.info("Storing embeddings in Qdrant...")
            stored_count = self.qdrant_manager.store_embeddings(list(valid_chunks), list(valid_embeddings))
            
            logger.info("Ingestion pipeline completed successfully!")
            
            return {
                "total_chunks_embedded": stored_count,
                "collection_created_or_updated": True,
                "skipped_content": skipped_content,
                "errors": errors
            }
        else:
            logger.error("No valid embeddings to store")
            return {
                "total_chunks_embedded": 0,
                "collection_created_or_updated": True,  # Collection was still created/verified
                "skipped_content": skipped_content,
                "errors": errors + ["No valid embeddings generated"]
            }


def main():
    # Get configuration from settings (to match the original working script)
    from src.config.settings import settings

    # Create and run the ingestion agent
    agent = DataIngestionAgent(
        cohere_api_key=settings.cohere_api_key,
        qdrant_url=settings.qdrant_url,
        qdrant_api_key=settings.qdrant_api_key,
        collection_name=settings.qdrant_collection_name
    )

    # Use the source URL from environment or default
    source_base_url = os.getenv("SOURCE_BASE_URL", "https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app")

    # Construct the sitemap URL
    sitemap_url = f"{source_base_url}/sitemap.xml"
    
    print("Starting AI Data Ingestion and Vectorization Agent...")
    print(f"Crawling website: {sitemap_url}")
    print(f"Storing in Qdrant collection: {settings.qdrant_collection_name}")
    print("-" * 60)

    result = agent.ingest_website(sitemap_url)
    
    print("\nINGESTION SUMMARY:")
    print(f"Total chunks embedded: {result['total_chunks_embedded']}")
    print(f"Collection updated: {result['collection_created_or_updated']}")
    print(f"Skipped content: {len(result['skipped_content'])}")
    print(f"Errors occurred: {len(result['errors'])}")

    if result['skipped_content']:
        print("\nSKIPPED CONTENT DETAILS:")
        for item in result['skipped_content'][:5]:  # Show first 5
            print(f"  - {item['url']} (reason: {item['reason']})")
        if len(result['skipped_content']) > 5:
            print(f"  ... and {len(result['skipped_content']) - 5} more")

    if result['errors']:
        print("\nERRORS:")
        for error in result['errors'][:5]:  # Show first 5
            print(f"  - {error}")
        if len(result['errors']) > 5:
            print(f"  ... and {len(result['errors']) - 5} more")

    print(f"\nProcess completed! Data is now available for semantic search and RAG.")


if __name__ == "__main__":
    main()