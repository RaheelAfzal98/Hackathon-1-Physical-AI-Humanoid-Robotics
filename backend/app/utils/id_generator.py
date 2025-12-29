import hashlib
from typing import Optional

def generate_chunk_id(url: str, content: str, chunk_index: int) -> str:
    """
    Generate a deterministic ID for a content chunk using SHA256 hash
    of URL + content snippet + chunk index
    """
    content_to_hash = f"{url}|{content[:100]}|{chunk_index}"
    return hashlib.sha256(content_to_hash.encode()).hexdigest()

def generate_content_hash(content: str) -> str:
    """
    Generate a hash of the content for change detection
    """
    return hashlib.sha256(content.encode()).hexdigest()

def generate_page_hash(content: str) -> str:
    """
    Generate a hash of the entire page content for change detection
    """
    return hashlib.sha256(content.encode()).hexdigest()