# End-to-End User Flow Checklist for Physical AI & Humanoid Robotics Textbook

This checklist covers the critical end-to-end user flows, integrating authentication, personalization, RAG assistance, and translation features.

## Authentication Flow
- [ ] User can successfully sign up with a new username and password.
- [ ] User receives appropriate feedback (success/error) upon signup.
- [ ] User can successfully log in with registered credentials.
- [ ] User receives appropriate feedback (success/error) upon login.
- [ ] User can remain logged in across sessions (e.g., via refresh tokens, if implemented).
- [ ] Invalid credentials result in an appropriate error message.

## Personalized Chapter Experience
- [ ] Logged-in user is recognized upon visiting the site.
- [ ] Personalized recommendations are displayed on the homepage or a dedicated section.
- [ ] Recommendations are relevant based on mock user data (e.g., "you showed interest in X, so here's Y").
- [ ] User's reading progress is tracked for a chapter (e.g., "Continue Reading" button).
- [ ] User can update personalization settings (e.g., preferred learning style, topics of interest).
- [ ] Changes to personalization settings are persisted and reflected in recommendations.

## RAG AI Assistant Interaction
- [ ] "Ask the Textbook" widget/button is visible and interactive.
- [ ] User can input a query into the RAG AI assistant.
- [ ] RAG assistant returns relevant textbook snippets or answers based on query.
- [ ] Response includes source chapter/segment information.
- [ ] Mock responses are handled gracefully (if actual RAG is not fully implemented).
- [ ] Clear error messages are displayed if the RAG query fails.

## Urdu Translation Toggle
- [ ] Urdu Translation button/toggle is visible on chapter pages.
- [ ] Clicking the toggle switches the chapter content to Urdu.
- [ ] Clicking the toggle again reverts content to English (or original language).
- [ ] The translation is applied dynamically without a full page reload.
- [ ] Basic elements (headings, paragraphs) are translated (even if mock translation).
- [ ] Language preference is persisted across pages or sessions (if implemented).

## Integration & Data Consistency
- [ ] User authentication status correctly influences personalization features.
- [ ] Reading progress updates are linked to the authenticated user.
- [ ] RAG queries can optionally filter results based on user's progress or preferences.
- [ ] Translation toggle does not interfere with other dynamic content or styling.

## Manual Testing Notes
- [ ] Test on different devices/screen sizes for responsiveness.
- [ ] Check console for any unexpected errors during user interactions.
- [ ] Verify API calls to the backend are correctly made and responses handled.
