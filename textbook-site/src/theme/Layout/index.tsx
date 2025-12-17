import React, {type ReactNode, useEffect} from 'react'; // Import useEffect
import clsx from 'clsx';
import ErrorBoundary from '@docusaurus/ErrorBoundary';
import {
  PageMetadata,
  SkipToContentFallbackId,
  ThemeClassNames,
} from '@docusaurus/theme-common';
import {useKeyboardNavigation} from '@docusaurus/theme-common/internal';
import SkipToContent from '@theme/SkipToContent';
import AnnouncementBar from '@theme/AnnouncementBar';
import Navbar from '@theme/Navbar';
import Footer from '@theme/Footer';
import LayoutProvider from '@theme/Layout/Provider';
import ErrorPageContent from '@theme/ErrorPageContent';
import type {Props} from '@theme/Layout';
import styles from './styles.module.css';
import Chatbot from '../../components/Chatbot'; // Import the Chatbot component
import { useLocation } from '@docusaurus/router'; // Import useLocation
import { useActivePluginAndVersion } from '@docusaurus/plugin-content-docs/client'; // Import useActivePluginAndVersion

const BASE_URL = 'http://localhost:8000'; // RAG Backend URL
const USER_ID = 1; // Hardcoded user ID for POC

export default function Layout(props: Props): ReactNode {
  const {
    children,
    noFooter,
    wrapperClassName,
    // Not really layout-related, but kept for convenience/retro-compatibility
    title,
    description,
  } = props;

  useKeyboardNavigation();
  const location = useLocation(); // Get current location
  const activeDocContext = useActivePluginAndVersion(); // Get docs context

  useEffect(() => {
    // Only track if it's a documentation page
    if (activeDocContext && activeDocContext.activeDoc) {
      const pathParts = location.pathname.split('/');
      // Expected URL format: /docs/chapter-id/page-id
      // We are looking for the part after /docs/ and before the next slash,
      // e.g., '01-ros2' from '/docs/01-ros2/ros2-basics'
      const docsIndex = pathParts.indexOf('docs');
      if (docsIndex !== -1 && pathParts.length > docsIndex + 1) {
        const chapterId = pathParts[docsIndex + 1];
        
        const trackProgress = async () => {
          try {
            const response = await fetch(`${BASE_URL}/personalize/progress/${USER_ID}`, {
              method: 'POST',
              headers: {
                'Content-Type': 'application/json',
              },
              body: JSON.stringify({ chapter_id: chapterId, completion_percentage: 0 }), // Simple tracking
            });

            if (!response.ok) {
              console.error(`Error tracking progress: ${response.status} - ${await response.text()}`);
            } else {
              console.log(`Tracked progress for user ${USER_ID} on chapter: ${chapterId}`);
            }
          } catch (error) {
            console.error('Failed to send progress update:', error);
          }
        };
        trackProgress();
      }
    }
  }, [location.pathname, activeDocContext]); // Re-run effect when pathname or activeDoc changes

  return (
    <LayoutProvider>
      <PageMetadata title={title} description={description} />

      <SkipToContent />

      <AnnouncementBar />

      <Navbar />

      <div
        id={SkipToContentFallbackId}
        className={clsx(
          ThemeClassNames.layout.main.container,
          ThemeClassNames.wrapper.main,
          styles.mainWrapper,
          wrapperClassName,
        )}>
        <ErrorBoundary fallback={(params) => <ErrorPageContent {...params} />}>
          {children}
        </ErrorBoundary>
      </div>

      {!noFooter && <Footer />}
      <Chatbot /> {/* Render the Chatbot component here */}
    </LayoutProvider>
  );
}
