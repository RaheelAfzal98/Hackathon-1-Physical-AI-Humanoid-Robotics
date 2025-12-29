from dataclasses import dataclass
from typing import Optional
from datetime import datetime

@dataclass
class ContentChunk:
    """
    Represents a segment of cleaned text from the source website,
    with associated semantic embedding, source URL, and timestamp
    """
    id: str  # Unique identifier for the chunk (SHA256 hash of URL + content snippet + chunk index)
    text_content: str  # The cleaned text content of the chunk
    source_url: str  # The URL from which this content was extracted
    created_at: datetime  # Timestamp when this chunk was created
    updated_at: datetime  # Timestamp when this chunk was last updated
    chunk_index: int  # Position of this chunk within the original document
    total_chunks: int  # Total number of chunks from the original document
    hash: str  # Hash of the content for change detection
    title: Optional[str] = None  # Optional title of the content

    def __post_init__(self):
        """Validate the content chunk after initialization"""
        if not self.text_content:
            raise ValueError("text_content must not be empty")

        if self.chunk_index < 0:
            raise ValueError("chunk_index must be >= 0")

        if self.total_chunks <= 0:
            raise ValueError("total_chunks must be > 0")

        if self.total_chunks < self.chunk_index:
            raise ValueError("total_chunks must be >= chunk_index")