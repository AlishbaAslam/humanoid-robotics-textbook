import React from 'react';
import ChatWidget from '../components/ChatInterface/ChatWidget';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}