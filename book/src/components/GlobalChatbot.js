import React from 'react';
import Chatbot from '@site/src/components/Chatbot';

// This component wraps the entire app and adds the chatbot
const GlobalChatbot = (props) => {
  return (
    <>
      {props.children}
      <Chatbot />
    </>
  );
};

export default GlobalChatbot;