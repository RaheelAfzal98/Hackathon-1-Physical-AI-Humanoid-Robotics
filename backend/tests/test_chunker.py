import pytest
from app.services.chunker import chunk_text


class TestChunker:
    """Unit tests for the chunker service"""
    
    def test_basic_chunking(self):
        """Test basic text chunking functionality"""
        text = "This is sentence one. This is sentence two. This is sentence three. This is sentence four."
        chunks = chunk_text(text, chunk_size=30, chunk_overlap=5)
        
        assert len(chunks) > 0
        for chunk in chunks:
            assert 'text' in chunk
            assert 'chunk_index' in chunk
            assert 'total_chunks' in chunk
            assert len(chunk['text']) <= 30  # Should respect size limit
    
    def test_chunk_indices(self):
        """Test that chunk indices and total counts are correct"""
        text = "A. B. C. D. E."
        chunks = chunk_text(text, chunk_size=5, chunk_overlap=1)
        
        # Verify that indices are sequential and total is correct
        for i, chunk in enumerate(chunks):
            assert chunk['chunk_index'] == i
            assert chunk['total_chunks'] == len(chunks)
    
    def test_empty_text(self):
        """Test chunking behavior with empty text"""
        chunks = chunk_text("", chunk_size=50, chunk_overlap=5)
        assert len(chunks) == 0