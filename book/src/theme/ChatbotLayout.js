import React from 'react';
import Chatbot from '@site/src/components/Chatbot';
import Layout from '@theme/Layout';

export default function ChatbotLayout(props) {
  return (
    <Layout {...props}>
      {props.children}
      <Chatbot />
    </Layout>
  );
}