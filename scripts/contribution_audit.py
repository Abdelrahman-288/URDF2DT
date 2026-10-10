"""Read-only Git contribution audit; remote results use the last fetched refs."""
import argparse
import json
from pathlib import Path
import subprocess


def git(repo, *args, optional=False):
    result = subprocess.run(["git", "-C", str(repo), *args], text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode and not optional:
        raise RuntimeError(result.stderr.strip())
    return result.stdout.strip() if result.returncode == 0 else None


def audit(repo, expected_email=None):
    branch = git(repo, "branch", "--show-current") or "(detached HEAD)"
    default = git(repo, "symbolic-ref", "--short", "refs/remotes/origin/HEAD", optional=True)
    warnings = ["Remote branch results reflect the last fetch; this script never fetches or changes files."]
    if default is None:
        warnings.append("Default branch unknown: set origin/HEAD after fetching, or inspect GitHub.")
    identity = git(repo, "var", "GIT_AUTHOR_IDENT", optional=True)
    email = identity.split("<", 1)[1].split(">", 1)[0] if identity and "<" in identity else None
    if not identity:
        warnings.append("No usable author identity configured.")
    elif expected_email and email.casefold() != expected_email.casefold():
        warnings.append("Effective author email differs from --expected-email.")
    warnings.append("Email ownership and graph eligibility require GitHub verification; names alone prove neither.")
    refs = git(repo, "for-each-ref", "--format=%(refname:short)", "refs/heads").splitlines()
    branches = []
    for ref in refs:
        upstream = git(repo, "rev-parse", "--abbrev-ref", ref + "@{upstream}", optional=True)
        branches.append({"branch": ref, "upstream": upstream,
                         "ahead_of_upstream": int(git(repo, "rev-list", "--count", f"{upstream}..{ref}")) if upstream else None,
                         "not_on_default": int(git(repo, "rev-list", "--count", f"{default}..{ref}")) if default else None})
    emails = git(repo, "log", "--all", "--format=%ae").splitlines()
    return {"repository": str(Path(repo).resolve()), "branch": branch,
            "default_ref": default, "author_identity": identity,
            "email_config_sources": git(repo, "config", "--show-origin", "--get-all", "user.email", optional=True),
            "status": git(repo, "status", "--porcelain=v1", "--untracked-files=all").splitlines(),
            "local_commits_not_on_any_remote": git(repo, "log", "--oneline", "--branches", "--not", "--remotes").splitlines(),
            "branches": branches,
            "refs_not_merged_into_default": git(repo, "for-each-ref", f"--no-merged={default}", "--format=%(refname:short)", "refs/heads", "refs/remotes").splitlines() if default else None,
            "historical_author_emails": {e: emails.count(e) for e in sorted(set(emails))},
            "warnings": warnings}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("repositories", nargs="*", type=Path, default=[Path.cwd()])
    parser.add_argument("--expected-email", help="An email you have verified on your GitHub account")
    args = parser.parse_args()
    reports = []
    for repo in args.repositories:
        try:
            reports.append(audit(repo, args.expected_email))
        except (OSError, RuntimeError) as exc:
            reports.append({"repository": str(repo), "error": str(exc)})
    print(json.dumps(reports, indent=2))
    return int(any("error" in report for report in reports))


if __name__ == "__main__":
    raise SystemExit(main())
