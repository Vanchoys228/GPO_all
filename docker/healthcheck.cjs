fetch(`http://127.0.0.1:${process.argv[2]}/ready`, { signal: AbortSignal.timeout(3000) })
  .then(response => { if (!response.ok) process.exitCode = 1; })
  .catch(() => { process.exitCode = 1; });
