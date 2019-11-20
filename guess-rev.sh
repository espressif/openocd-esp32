#!/bin/sh
#
# This scripts adds local version information from the version
# control systems git, mercurial (hg) and subversion (svn).
#
# Copied from Linux 2.6.32 scripts/setlocalversion and modified
# slightly to work better for OpenOCD.
#

usage() {
	echo "Usage: $0 [srctree]" >&2
	exit 1
}

cd "${1:-.}" || usage

# Check for git and a git repo, use 'git describe' output as version.
desc=`git describe --always --tags --dirty --abbrev=8 2>/dev/null`
if [ -n "$desc" ]; then
	printf '%s' $desc
	exit
fi

# Check for mercurial and a mercurial repo.
if hgid=`hg id 2>/dev/null`; then
	tag=`printf '%s' "$hgid" | cut -d' ' -f2`

	# Do we have an untagged version?
	if [ -z "$tag" -o "$tag" = tip ]; then
		id=`printf '%s' "$hgid" | sed 's/[+ ].*//'`
		printf '%s%s' -hg "$id"
	fi

	# Are there uncommitted changes?
	# These are represented by + after the changeset id.
	case "$hgid" in
		*+|*+\ *) printf '%s' -dirty ;;
	esac

	# All done with mercurial
	exit
fi

# Check for svn and a svn repo.
if rev=`svn info 2>/dev/null | grep '^Last Changed Rev'`; then
	rev=`echo $rev | awk '{print $NF}'`
	printf -- '-svn%s' "$rev"

	# All done with svn
	exit
fi

# There's no recognized repository; we must be a snapshot.
printf -- '-snapshot'
