import os
from dotenv import load_dotenv

print("Current working directory:", os.getcwd())
print("Files in current directory:", os.listdir('.'))

print("\nEnvironment variable before loading .env:")
print("QDRANT_URL:", os.environ.get("QDRANT_URL"))
print("QDRANT_API_KEY:", os.environ.get("QDRANT_API_KEY"))

# Load environment variables with override
load_dotenv(override=True)

print("\nEnvironment variables after loading .env with override:")
print("QDRANT_URL:", os.environ.get("QDRANT_URL"))
print("QDRANT_API_KEY:", os.environ.get("QDRANT_API_KEY"))
print()

# Read the .env file directly to see its content
with open('.env', 'r') as f:
    env_content = f.read()
    print("Content of .env file:")
    print(env_content)
    print()

# Now try to load using Pydantic Settings
from src.config.settings import settings

print("Settings loaded from Pydantic Settings:")
print("QDRANT_URL:", settings.qdrant_url)
print("QDRANT_API_KEY:", settings.qdrant_api_key)
print()

# Check if the values are different
if os.environ.get("QDRANT_URL") != settings.qdrant_url:
    print("WARNING: Environment variable and settings don't match!")
else:
    print("Environment variable and settings match.")