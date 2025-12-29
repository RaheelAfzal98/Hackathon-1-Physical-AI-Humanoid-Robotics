import React, { useState, useEffect, useRef } from 'react';
import './Chatbot.css';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, text: 'Hello! I\'m your AI assistant for the Physical AI & Humanoid Robotics textbook. How can I help you today? (Note: Backend requires API keys to function)', sender: 'bot' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user'
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API to get a response
      const response = await fetch('http://localhost:8000/agent/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query_text: inputValue,
          session_id: 'session-' + Date.now(),
          user_context: {},
          response_options: {}
        }),
      });

      if (response.ok) {
        const data = await response.json();
        const botMessage = {
          id: Date.now() + 1,
          text: data.content || 'I couldn\'t find a good answer to your question. Please try rephrasing.',
          sender: 'bot'
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        // Try to get more specific error information
        let errorMessage = 'Sorry, I encountered an error processing your request. Please try again.';

        try {
          const errorData = await response.json();
          if (errorData.detail) {
            // Check if it's an API key issue
            if (errorData.detail.toLowerCase().includes('api') &&
                (errorData.detail.toLowerCase().includes('key') ||
                 errorData.detail.toLowerCase().includes('authentication'))) {
              errorMessage = 'The backend requires valid API keys to function. Please ensure your API keys are properly configured in the backend .env file.';
            } else {
              errorMessage = `Error: ${errorData.detail}`;
            }
          }
        } catch (e) {
          // If we can't parse the error response, use a generic message
          console.error('Could not parse error response:', e);
        }

        const botMessage = {
          id: Date.now() + 1,
          text: errorMessage,
          sender: 'bot'
        };
        setMessages(prev => [...prev, botMessage]);
      }
    } catch (error) {
      console.error('Error:', error);
      const botMessage = {
        id: Date.now() + 1,
        text: 'Note: The backend RAG system requires valid API keys to function. To use the full AI assistant functionality, please configure your Cohere and Qdrant API keys in the backend .env file.',
        sender: 'bot'
      };
      setMessages(prev => [...prev, botMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div className="chatbot-container">
      {isOpen ? (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <div className="header-content">
              <h3>AI Assistant</h3>
              <div className="status-indicator">
                <span className="status-dot"></span>
                <span className="status-text">Backend required</span>
              </div>
            </div>
            <button className="chatbot-close" onClick={() => setIsOpen(false)}>
              ×
            </button>
          </div>
          <div className="chatbot-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.sender}`}
              >
                <div className="message-text">{message.text}</div>
              </div>
            ))}
            {isLoading && (
              <div className="message bot">
                <div className="message-text typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <div className="chatbot-input">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about Physical AI & Humanoid Robotics..."
              rows="2"
            />
            <button
              onClick={sendMessage}
              disabled={isLoading || !inputValue.trim()}
              className="send-button"
            >
              Send
            </button>
          </div>
        </div>
      ) : (
        <button className="chatbot-toggle" onClick={() => setIsOpen(true)}>
          💬
        </button>
      )}
    </div>
  );
};

export default Chatbot;