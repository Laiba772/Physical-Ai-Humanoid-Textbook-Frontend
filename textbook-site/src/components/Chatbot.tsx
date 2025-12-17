// Chatbot.tsx
import React, { useEffect, useMemo, useRef, useState } from "react";
import clsx from "clsx";

const BASE_URL = "http://localhost:8000"; // RAG Backend URL

type Msg = { id: string; sender: "user" | "bot"; text: string; meta?: { hint?: string } };

const uid = () => Math.random().toString(16).slice(2) + Date.now().toString(16);

const Chatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Msg[]>([]);
  const [input, setInput] = useState("");
  const [isLoading, setIsLoading] = useState(false);

  const scrollRef = useRef<HTMLDivElement | null>(null);
  const inputRef = useRef<HTMLInputElement | null>(null);

  const quickPrompts = useMemo(
    () => [
      "What is Physical AI (definition + key idea)?",
      "Explain the difference: perception vs control (in one paragraph).",
      "Give 2 real-world examples of humanoid robotics tasks.",
    ],
    []
  );

  useEffect(() => {
    if (isOpen) {
      requestAnimationFrame(() => inputRef.current?.focus());
    }
  }, [isOpen]);

  useEffect(() => {
    if (!scrollRef.current) return;
    scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
  }, [messages, isLoading]);

  const toggleChat = () => setIsOpen((v) => !v);

  const append = (next: Msg) => setMessages((prev) => [...prev, next]);

  const send = async (text: string) => {
    const q = text.trim();
    if (!q || isLoading) return;

    const userMsg: Msg = { id: uid(), sender: "user", text: q };
    append(userMsg);

    setInput("");
    setIsLoading(true);

    try {
      const res = await fetch(`${BASE_URL}/search`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ query: q, top_k: 3 }),
      });

      if (!res.ok) throw new Error(`HTTP ${res.status}`);

      const data = await res.json();
      const botText =
        Array.isArray(data) && data.length
          ? data
              .map((r: any) => (r?.content_snippet ? `• ${r.content_snippet}` : null))
              .filter(Boolean)
              .join("\n")
          : "I don't know.";

      append({ id: uid(), sender: "bot", text: botText });
    } catch (err) {
      console.error("RAG request error:", err);
      append({ id: uid(), sender: "bot", text: "Sorry — I hit a problem fetching the answer. Please try again." });
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {/* Floating action button (stylish) */}
      <button
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
        className="fixed hover:cursor-pointer bottom-6 right-6 z-50 flex items-center justify-center rounded-full
          border border-white/30 bg-linear-to-br from-violet-600 via-indigo-600 to-sky-500
          text-white shadow-[0_18px_60px_rgba(15,23,42,0.45)] transition-transform hover:scale-105 focus:outline-none focus:ring-2 focus:ring-white/90"
      >
        <span className="sr-only">{isOpen ? "Close" : "Open"} assistant</span>
        <svg className="h-12 w-12" viewBox="0 0 24 24" fill="none" aria-hidden="true">
          <path
            d="M8 12h.01M12 12h.01M16 12h.01M21 12c0 4.418-4.03 8-9 8a9.86 9.86 0 0 1-4.255-.949L3 20l1.395-3.72C3.512 15.042 3 13.574 3 12c0-4.418 4.03-8 9-8s9 3.582 9 8Z"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
        </svg>
      </button>

      {/* Chat panel */}
      <div
        className={clsx(
          "fixed bottom-20 right-6 z-50 w-[340px] max-w-[92vw] rounded-2xl",
          "backdrop-blur-xl bg-white/85 border border-white/30",
          "shadow-[0_24px_80px_rgba(15,23,42,0.55)] transition-all duration-300",
          isOpen ? "translate-y-0 opacity-100" : "translate-y-5 opacity-0 pointer-events-none"
        )}
        role="dialog"
        aria-label="Textbook assistant chat"
      >
        {/* Header */}
        <div className="flex items-center justify-between gap-2 rounded-2xl rounded-b-none px-4 py-3">
          <div className="min-w-0">
            <div className="flex items-center gap-2">
              <div className="h-2 w-2 rounded-full bg-emerald-400 shadow-[0_0_0_6px_rgba(16,185,129,0.15)]" />
              <div className="text-sm font-semibold tracking-tight text-slate-900">
                Textbook Assistant
              </div>
            </div>
            <div className="text-xs text-slate-600/90">RAG-powered · answers from your book</div>
          </div>
          <button
            onClick={toggleChat}
            className="inline-flex items-center justify-center rounded-lg p-1.5 text-slate-700 hover:bg-slate-100 focus:outline-none focus:ring-2 focus:ring-slate-300"
            aria-label="Close chat"
          >
            <svg className="h-5 w-5" viewBox="0 0 24 24" fill="none" aria-hidden="true">
              <path d="M6 18L18 6M6 6l12 12" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
            </svg>
          </button>
        </div>

        {/* Quick prompts (optional flavor) */}
        <div className="px-4 pb-2">
          <div className="flex flex-wrap gap-2">
            {quickPrompts.map((p) => (
              <button
                key={p}
                onClick={() => send(p)}
                className="text-[11px] font-medium rounded-full border border-slate-200 bg-white/80 px-3 py-1.5
                  hover:bg-slate-900 hover:text-white transition-colors"
              >
                {p}
              </button>
            ))}
          </div>
        </div>

        {/* Messages */}
        <div ref={scrollRef} className="min-h-44 max-h-[280px] overflow-y-auto px-4 pb-3">
          {messages.map((m) => {
            const isUser = m.sender === "user";
            return (
              <div key={m.id} className={clsx("mb-3 flex gap-2", isUser ? "justify-end" : "justify-start")}>
                {!isUser && (
                  <div className="mt-0.5 h-7 w-7 rounded-full bg-linear-to-br from-sky-500 to-violet-500" />
                )}
                <div
                  className={clsx(
                    "max-w-[88%] rounded-2xl px-3 py-2 text-sm leading-relaxed",
                    isUser
                      ? "bg-slate-900 text-white shadow-sm"
                      : "bg-white/90 text-slate-900 border border-slate-200"
                  )}
                >
                  {m.text}
                </div>
                {isUser && <div className="mt-0.5 h-7 w-7 rounded-full bg-slate-800" />}
              </div>
            );
          })}

          {isLoading && (
            <div className="mb-3 flex items-center gap-2">
              <div className="h-7 w-7 rounded-full bg-linear-to-br from-sky-500 to-violet-500" />
              <div className="rounded-2xl bg-white/90 border border-slate-200 px-3 py-2 text-sm text-slate-700">
                Thinking<span className="animate-pulse">…</span>
              </div>
            </div>
          )}
        </div>

        {/* Composer */}
        <div className="border-t border-white/30 bg-white/60 px-3 py-3">
          <div className="flex gap-2">
            <input
              ref={inputRef}
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyDown={(e) => {
                if (e.key === "Enter" && !e.shiftKey) send(input);
              }}
              disabled={isLoading}
              placeholder="Ask a chapter-level question… (Enter to send)"
              className="flex-1 rounded-xl border border-slate-200 bg-white/80 px-3 py-2 text-sm outline-none
                focus:border-sky-400 focus:ring-2 focus:ring-sky-200 disabled:opacity-60"
            />
            <button
              onClick={() => send(input)}
              disabled={!input.trim() || isLoading}
              className="rounded-xl px-4 py-2 text-sm font-semibold text-white 
                bg-linear-to-r from-violet-600 via-indigo-600 to-sky-500
                hover:brightness-105 disabled:opacity-55"
            >
              Send
            </button>
          </div>
        </div>
      </div>
    </>
  );
};

export default Chatbot;
