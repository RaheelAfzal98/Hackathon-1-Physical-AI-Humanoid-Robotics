"""
Simple script to update .env file to use local Qdrant settings
"""

import os

def update_env_file():
    """Update the .env file to use local Qdrant."""
    env_path = '../.env'
    
    # Read current .env content
    with open(env_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    # Update Qdrant settings to use local instance
    updated_lines = []
    for line in lines:
        if line.startswith('QDRANT_URL='):
            updated_lines.append('QDRANT_URL="http://localhost:6333"\n')
        elif line.startswith('QDRANT_API_KEY='):
            updated_lines.append('QDRANT_API_KEY=""\n')  # Local Qdrant doesn't need API key
        else:
            updated_lines.append(line)
    
    # Write updated content back
    with open(env_path, 'w', encoding='utf-8') as f:
        f.writelines(updated_lines)
    
    print("Updated .env file to use local Qdrant")

def main():
    print("Updating .env file to use local Qdrant settings...")
    update_env_file()
    
    print("\nDone! The .env file has been updated to use a local Qdrant instance.")
    print("To use local Qdrant, you'll need to install and run it separately.")
    print("You can install Qdrant using one of these methods:")
    print("1. Docker: docker run -d --name qdrant -p 6333:6333 qdrant/qdrant")
    print("2. Download from: https://qdrant.tech/documentation/quick-start/")
    print("3. Use Qdrant Cloud: https://cloud.qdrant.io/")
    print("\nAfter setting up Qdrant, restart your backend server to apply the changes.")

if __name__ == "__main__":
    main()