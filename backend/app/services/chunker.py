from typing import List, Dict
import re

def chunk_text(text_content: str, chunk_size: int = 512, chunk_overlap: int = 50) -> List[Dict[str, any]]:
    """
    Split text content into overlapping chunks of specified size
    Maintains semantic coherence across chunk boundaries
    """
    if not text_content:
        return []

    # Split text into sentences to maintain semantic coherence
    sentences = re.split(r'[.!?]+', text_content)

    chunks = []
    current_chunk = ""
    chunk_index = 0

    for sentence in sentences:
        sentence = sentence.strip()
        if not sentence:
            continue

        # Check if adding this sentence would exceed chunk size
        if len(current_chunk) + len(sentence) <= chunk_size:
            current_chunk += " " + sentence
        else:
            # If current chunk is not empty, save it
            if current_chunk.strip():
                chunks.append({
                    'text': current_chunk.strip(),
                    'chunk_index': chunk_index,
                    'total_chunks': 0  # Will update this after all chunks are created
                })
                chunk_index += 1

            # Start a new chunk, potentially with overlap
            if len(sentence) > chunk_size:
                # If the sentence is longer than chunk_size, split it by chunks
                sub_chunks = _split_large_sentence(sentence, chunk_size)
                for sc in sub_chunks[:-1]:  # Add all but the last piece to chunks
                    chunks.append({
                        'text': sc,
                        'chunk_index': chunk_index,
                        'total_chunks': 0
                    })
                    chunk_index += 1

                # Make the last sub-chunk the current chunk (to potentially add more to it)
                current_chunk = sub_chunks[-1]
            else:
                # Start a new chunk with some overlap from the previous chunk
                if chunk_overlap > 0 and chunks:
                    # Get some text from the end of the last chunk to maintain context
                    prev_chunk_text = chunks[-1]['text']
                    overlap_text = prev_chunk_text[-chunk_overlap:] if len(prev_chunk_text) > chunk_overlap else prev_chunk_text
                    current_chunk = overlap_text + " " + sentence
                else:
                    current_chunk = sentence

    # Add the final chunk if it's not empty
    if current_chunk.strip():
        chunks.append({
            'text': current_chunk.strip(),
            'chunk_index': chunk_index,
            'total_chunks': 0
        })

    # Update the total_chunks value for all chunks
    total_chunks = len(chunks)
    for chunk in chunks:
        chunk['total_chunks'] = total_chunks

    return chunks

def _split_large_sentence(sentence: str, chunk_size: int) -> List[str]:
    """
    Split a sentence that is longer than the chunk size into smaller pieces
    """
    if len(sentence) <= chunk_size:
        return [sentence]

    pieces = []
    for i in range(0, len(sentence), chunk_size):
        pieces.append(sentence[i:i+chunk_size])

    return pieces