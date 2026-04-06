# Contributing to SLAM Agent

Thanks for using and contributing to SLAM Agent!

## Share What Your Agent Learns

The most valuable contribution you can make is **committing your agent's learnings and hardware profiles back to the repo**. This helps everyone in the community avoid re-solving the same problems.

### What to contribute

- **Hardware profiles** (`docs/profiles/`) — YAML configs for your specific hardware combo (flight controller, LiDAR, camera, compute platform)
- **Learned data** (`docs/learned/`) — solutions, known-good configs, and hardware profile entries your agent has built up during real deployments

### How to contribute learnings

1. **During a session**, the agent automatically saves learnings to `docs/learned/` and profiles to `docs/profiles/`.

2. **Commit them** using the MCP tool:
   ```
   commit_learning
   ```
   Or manually:
   ```bash
   git add docs/learned/ docs/profiles/
   git commit -m "Add learnings for <your hardware setup>"
   ```

3. **Open a pull request** against `main`.

### Guidelines for learnings

- Include your hardware fingerprint (platform, sensors, firmware versions)
- Mark profiles as `validated: true` only after successful flight testing
- Document any workarounds with the failure mode that triggered them
- Remove any site-specific data (IP addresses, WiFi credentials, etc.) before committing

## Code Contributions

1. Fork the repo and create a feature branch
2. Keep changes focused — one feature or fix per PR
3. Test on real hardware if possible, or document what was tested
4. Open a PR with a clear description of the change

## Bug Reports

Open an issue with:
- Your hardware setup (platform, sensors, flight controller)
- Steps to reproduce
- Relevant logs or error output

## License

By contributing, you agree that your contributions will be licensed under the [GPL v3](LICENSE).
