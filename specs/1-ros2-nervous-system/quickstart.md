# Quickstart: ROS 2 Nervous System Module

## 1. Setup Development Environment
```bash
# Install ROS 2 Humble Hawksbill
# Set up Python 3.10 environment
# Install required dependencies
pip install rclpy docusaurus fastapi uvicorn
```

## 2. Build the Book
```bash
# Navigate to book directory
cd book/
# Install dependencies
npm install
# Run local server
npm start
```

## 3. Run RAG System Locally
```bash
# Start the backend
cd src/rag/backend/fastapi
uvicorn main:app --reload
```

## 4. Content Development
- Create Markdown files in the docs directory following the module structure
- Include code examples in designated code blocks
- Add diagrams and assets where needed
- Test all code examples for reproducibility

## 5. Testing
```bash
# Run Python tests
pytest tests/

# Run ROS 2 tests
colcon test

# Run integration tests for RAG system
python -m pytest tests/integration/
```

## 6. Deployment
- The book will be deployed to GitHub Pages
- RAG system will be deployed separately
- Updates to main branch trigger automatic deployment