workflow "Sync issues to JIRA" {
  on = "issues"
  resolves = ["Sync to JIRA"]
}

workflow "Sync issue and PR comments to JIRA" {
  on = "issue_comment"
  resolves = ["Sync to JIRA"]
}

workflow "Sync PRs to JIRA" {
  resolves = ["Sync to JIRA"]
  on = "pull_request"
}

action "Sync to JIRA" {
  uses = "espressif/github-actions/sync_issues_to_jira@master"
  secrets = [
    "GITHUB_TOKEN",
    "JIRA_USER",
    "JIRA_PASS",
    "JIRA_URL",
  ]
  env = {
    JIRA_PROJECT = "OCD"
  }
}
