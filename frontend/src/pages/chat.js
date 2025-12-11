import React from 'react';
import Layout from '@theme/Layout';
import RAGChatbot from '../components/RAGChatbot';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function ChatPage() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`RAG Chatbot - ${siteConfig.title}`}
      description="Ask questions about Physical AI and Humanoid Robotics textbook content">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <h1>Textbook RAG Chatbot</h1>
            <p>Ask questions about the Physical AI & Humanoid Robotics textbook content. The chatbot provides book-only answers with citations and confidence scores.</p>

            <RAGChatbot />

            <div style={{ marginTop: '2rem' }}>
              <h2>How to Use</h2>
              <ul>
                <li>Ask questions about any topic covered in the textbook</li>
                <li>Click on any text in the textbook and use the "Ask" functionality</li>
                <li>Responses include citations to specific textbook sections</li>
                <li>Confidence scores indicate the reliability of responses</li>
              </ul>

              <h2>Features</h2>
              <ul>
                <li>Book-only answers (no external information)</li>
                <li>Proper APA citations for all sources</li>
                <li>Confidence scoring for each response</li>
                <li>Response time under 800ms</li>
                <li>Support for selected-text questioning</li>
              </ul>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default ChatPage;