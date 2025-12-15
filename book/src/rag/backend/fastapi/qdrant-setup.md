# Qdrant Setup for RAG System

## Installation

### Option 1: Using Docker (Recommended)

```bash
# Pull the Qdrant image
docker pull qdrant/qdrant:latest

# Run Qdrant container
docker run -p 6333:6333 \
    -v $(pwd)/qdrant_storage:/qdrant/storage:z \
    -e QDRANT__SERVICE__HTTP_PORT=6333 \
    qdrant/qdrant:latest
```

### Option 2: Using Cloud Free Tier

1. Go to [Qdrant Cloud](https://cloud.qdrant.io/)
2. Sign up for a free account
3. Create a new collection
4. Note your API key and endpoint URL

## Configuration

Create a `.env` file with your Qdrant configuration:

```env
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your_api_key_here
```

## Testing the Connection

You can test the connection using Python:

```python
from qdrant_client import QdrantClient

# Connect to Qdrant
client = QdrantClient(url="http://localhost:6333")

# Check if connected
print(client.get_collections())
```

## Notes

- For development, use Docker option
- For production, use Cloud Free Tier (max 1M vectors, 2GB storage, 1M API calls/month)
- Make sure to add `.env` to your .gitignore file