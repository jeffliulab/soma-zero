# adapters — one thin translation layer per brain

An **adapter** translates the framework-neutral contract in [`interface/`](../interface/)
into one specific brain framework's wire protocol, and wraps the body as whatever kind of
endpoint that framework expects (an MCP server, an HTTP service, …).

Rules of the layer:

- **The body core never imports a brain framework.** All framework-specific code and
  protocol copies live here, quarantined per brain.
- **Adapters translate; they do not think.** No task logic, no retries, no judging success —
  an adapter only moves the contract's four flows (observation, action intent, execution
  result, progress) across the wire.
- **One subfolder per brain.** Adding support for a new framework means adding a folder
  here; the body core does not change.

## Current adapters

| Adapter | Brain | Status |
|---|---|---|
| [`anima/`](anima/) | [anima-zero](https://github.com/jeffliulab/anima-zero) — AWI over MCP | 📝 docs first (server lands here when perception/control have real behavior) |

Any MCP-host-style framework can reuse the `anima/` pattern nearly verbatim: the endpoint is
a standard MCP server, and nothing in it is ANIMA-private except two resource URI names.
