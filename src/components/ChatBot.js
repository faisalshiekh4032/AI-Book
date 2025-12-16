import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './ChatBot.module.css';

const ChatBot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [conversationId, setConversationId] = useState(null);
  const messagesEndRef = useRef(null);
  
  // FIX: Use Docusaurus context instead of process.env
  const {siteConfig} = useDocusaurusContext();
  const API_URL = siteConfig.customFields?.chatbotApiUrl || 'http://localhost:8000';

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    if (isOpen && messages.length === 0) {
      setMessages([{
        role: 'assistant',
        content: 'Hello! I\'m your AI course assistant. I can help you understand Physical AI and Humanoid Robotics concepts. What would you like to learn today?'
      }]);
    }
  }, [isOpen]);

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = inputValue.trim();
    setInputValue('');
    
    // Add user message to chat
    setMessages(prev => [...prev, { role: 'user', content: userMessage }]);
    setIsLoading(true);

    try {
      // Get selected text if any
      const selectedText = window.getSelection().toString();

      const response = await fetch(`${API_URL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: userMessage,
          selected_text: selectedText || null,
          conversation_id: conversationId
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to get response');
      }

      const data = await response.json();
      
      // Update conversation ID
      if (!conversationId) {
        setConversationId(data.conversation_id);
      }

      // Add assistant message to chat
      setMessages(prev => [...prev, { 
        role: 'assistant', 
        content: data.response,
        sources: data.sources 
      }]);
    } catch (error) {
      console.error('Error:', error);
      setMessages(prev => [...prev, { 
        role: 'assistant', 
        content: 'Sorry, the chatbot backend is not currently running. Please deploy the FastAPI backend to use this feature.' 
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const clearChat = () => {
    setMessages([{
      role: 'assistant',
      content: 'Hello! I\'m your AI course assistant. How can I help you today?'
    }]);
    setConversationId(null);
  };

  return (
    <>
      {/* Chat Button */}
      <button 
        className={styles.chatButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
        title="Ask Course Assistant"
      >
        {isOpen ? '✕' : '🤖'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <div>
              <h3>Course AI Assistant</h3>
              <p>Ask questions about Physical AI & Robotics</p>
            </div>
            <button 
              className={styles.clearButton}
              onClick={clearChat}
              title="Clear conversation"
            >
              🔄
            </button>
          </div>

          <div className={styles.messagesContainer}>
            {messages.map((msg, idx) => (
              <div 
                key={idx} 
                className={`${styles.message} ${styles[msg.role]}`}
              >
                <div className={styles.messageContent}>
                  {msg.content}
                </div>
                {msg.sources && msg.sources.length > 0 && (
                  <div className={styles.sources}>
                    <strong>Sources:</strong>
                    <ul>
                      {msg.sources.map((source, sidx) => (
                        <li key={sidx}>{source}</li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.messageContent}>
                  <div className={styles.typing}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className={styles.inputContainer}>
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about ROS 2, Isaac Sim, humanoids..."
              className={styles.input}
              rows={1}
              disabled={isLoading}
            />
            <button 
              onClick={handleSendMessage}
              className={styles.sendButton}
              disabled={isLoading || !inputValue.trim()}
              aria-label="Send message"
            >
              ➤
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatBot;
