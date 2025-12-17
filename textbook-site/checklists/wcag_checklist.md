# WCAG 2.1 Accessibility Checklist for Docusaurus Site

This checklist provides a basic overview of WCAG 2.1 guidelines to ensure the Docusaurus site is accessible. This is not exhaustive and should be supplemented with full accessibility audits.

## Perceivable

### Text Alternatives
- [ ] 1.1.1 Non-text Content: All non-text content (images, diagrams, multimedia) has text alternatives. (A)
- [ ] `alt` attributes are provided for `<img>` tags.
- [ ] For complex images, a longer description is provided (e.g., in adjacent text or `longdesc`).

### Time-based Media
- [ ] 1.2.1 Audio-only and Video-only (Prerecorded): Alternatives are provided for time-based media (e.g., transcript for audio, text track for video). (A)
- [ ] 1.2.2 Captions (Prerecorded): Captions are provided for all prerecorded audio content in synchronized media. (A)
- [ ] 1.2.3 Audio Description or Media Alternative (Prerecorded): An audio description or full text alternative is provided for prerecorded video content. (A)

### Adaptable
- [ ] 1.3.1 Info and Relationships: Information, structure, and relationships conveyed through presentation can be programmatically determined or are available in text. (A)
- [ ] Headings are used correctly (h1-h6 in logical order).
- [ ] Lists are marked up as lists (`<ul>`, `<ol>`).
- [ ] Data tables use proper markup (`<th>`, `<td>`, `<caption>`).
- [ ] 1.3.2 Meaningful Sequence: When the sequence of content affects its meaning, that sequence can be programmatically determined. (A)
- [ ] 1.3.3 Sensory Characteristics: Instructions do not rely solely on sensory characteristics (e.g., shape, size, visual location, orientation, or sound). (A)

### Distinguishable
- [ ] 1.4.1 Use of Color: Color is not used as the sole visual means of conveying information. (A)
- [ ] 1.4.3 Contrast (Minimum): Visual presentation of text and images of text has a contrast ratio of at least 4.5:1. (AA)
- [ ] 1.4.4 Resize text: Text can be resized without assistive technology up to 200 percent without loss of content or functionality. (AA)
- [ ] 1.4.10 Reflow: Content can be presented without loss of information or functionality, and without requiring scrolling in two dimensions for vertical scrolling content at a width equivalent to 320 CSS pixels. (AA)

## Operable

### Keyboard Accessible
- [ ] 2.1.1 Keyboard: All functionality of the content is operable through a keyboard interface without requiring specific timings for individual keystrokes. (A)
- [ ] All interactive elements (buttons, links, form fields) are reachable via keyboard (Tab key).
- [ ] 2.1.2 No Keyboard Trap: Keyboard focus is never trapped in a subsection of the content. (A)
- [ ] 2.1.4 Character Key Shortcuts: If a keyboard shortcut is implemented using only letter (uppercase and lowercase), punctuation, number, or symbol characters, then at least one of the following is true: Turn off, Remap, or Activated only on focus. (A)

### Enough Time
- [ ] 2.2.1 Timing Adjustable: Time limits are adjustable by the user. (A)
- [ ] 2.2.2 Pause, Stop, Hide: For moving, blinking, scrolling, or auto-updating information, there is a mechanism for the user to pause, stop, or hide it. (A)

### Seizures and Physical Reactions
- [ ] 2.3.1 Three Flashes or Below Threshold: Web pages do not contain anything that flashes more than three times in any one-second period, or the flash is below the general flash and red flash thresholds. (A)

### Navigable
- [ ] 2.4.1 Bypass Blocks: A mechanism is available to bypass blocks of content that are repeated on multiple Web pages. (A) (e.g., Skip to main content link)
- [ ] 2.4.2 Page Titled: Web pages have titles that describe topic or purpose. (A)
- [ ] 2.4.3 Focus Order: If a Web page can be navigated sequentially and the navigation sequences affect meaning or operation, focusable components receive focus in an order that preserves meaning and operability. (A)
- [ ] 2.4.4 Link Purpose (In Context): The purpose of each link can be determined from the link text alone or from the link text together with its programmatically determined link context. (A)
- [ ] 2.4.7 Focus Visible: Any user interface component that can be operated or navigated using a keyboard interface has a mode of operation where the keyboard focus indicator is visible. (AA)

## Understandable

### Readable
- [ ] 3.1.1 Language of Page: The default human language of each Web page can be programmatically determined. (A)
- [ ] 3.1.2 Language of Parts: The human language of each passage or phrase in the content can be programmatically determined. (AA)

### Predictable
- [ ] 3.2.1 On Focus: Changing the setting of any user interface component does not automatically cause a change of context unless the user has been advised of the behavior before using the component. (A)
- [ ] 3.2.2 On Input: Changing the setting of any user interface component does not automatically cause a change of context unless the user has been advised of the behavior before using the component. (A)
- [ ] 3.2.3 Consistent Navigation: Navigational mechanisms that are repeated on multiple Web pages within a set of Web pages occur in the same relative order each time they are repeated, unless a change is initiated by the user. (AA)
- [ ] 3.2.4 Consistent Identification: Components that have the same functionality within a set of Web pages are identified consistently. (AA)

### Input Assistance
- [ ] 3.3.1 Error Identification: If an input error is automatically detected, the item that is in error is identified and the error is described to the user in text. (A)
- [ ] 3.3.2 Labels or Instructions: Labels or instructions are provided when content requires user input. (A)

## Robust

### Compatible
- [ ] 4.1.1 Parsing: In content implemented using markup languages, elements have complete start and end tags, elements are nested according to their specifications, elements do not contain duplicate attributes, and any IDs are unique, except where the specifications allow these features. (A)
- [ ] 4.1.2 Name, Role, Value: For all user interface components (including but not limited to: form elements, links, and components generated by scripts), the name and role can be programmatically determined; states, properties, and values that can be set by the user can be programmatically set; and notification of changes to these items is available to user agents, including assistive technologies. (A)
