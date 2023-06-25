#!/usr/bin/env bash

##########################################################################
#  functions.sh - auxiliary functions
#
#  Created: Mon May 30 10:18:40 2016
#  Copyright  2016-2018  Tim Niemueller [www.niemueller.org]
##########################################################################

#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Library General Public License for more details.
#
#  Read the full text in the LICENSE.GPL file in the doc directory.


# Print failure message
print_fail()
{
	local COMPONENT=${1:-C}
	local MESSAGE=${2:-}
	>&2 echo -e "\033[1;31mFAIL|$COMPONENT:\033[0m $MESSAGE"
}

# Check if script dir (current directory assumed to be just that)
# is a git repo and that git is available
git_repo_check()
{
		if [ -d .git ]; then
				# Check for git
				if type -P git >/dev/null ; then
						return 0
				else
						print_fail "git_repo_check" "git not found"
						return 1
				fi
		else
				print_fail "git_repo_check" "directory $(dirname $(pwd)) is not a git repository"
				return 1
		fi
}

# Update the script/current directory git repository
git_repo_fetch()
{
		local REMOTE_URL=$(git config --get remote.origin.url)
		local REPO_DIR=$(pwd)

		echo "Fetching updates in $REPO_DIR"
		echo "  - remote: $REMOTE_URL"

		if ! git fetch; then
				print_fail "git_repo_pull" "Failed to fetch from remote $REMOTE_URL"
				return 1
		fi
}

# Stash any local modifications
git_repo_stash()
{
		if ! git stash; then
				print_fail "git_repo_stash" "Failed to stash changes"
				return 1
		fi
}

git_repo_changed()
{		
		local LOCAL_HEAD=$(git rev-parse @)      || return 2
		local REMOTE_HEAD=$(git rev-parse @{u})  || return 2
		local BASE_HEAD=$(git merge-base @ @{u}) || return 2

		[ -n "$LOCAL_HEAD" ]  || return 3
		[ -n "$REMOTE_HEAD" ] || return 3
		[ -n "$BASE_HEAD" ]   || return 3
		
		if [ $LOCAL_HEAD = $REMOTE_HEAD ]; then
				echo up-to-date
				return 1
		elif [ $LOCAL_HEAD = $BASE_HEAD ]; then
				echo pull
				return 0
		elif [ $REMOTE_HEAD = $BASE_HEAD ]; then
				echo push
				return 0
		else
				echo diverged
				return 0
		fi
}

git_ssh_keyscan ()
{
	URL=${1:-}
	if [[ "$URL" =~ ^(ssh://|git@)([^/]+)[/:]([^/]+)/(.+)\.git$ ]]; then
		HOST=${BASH_REMATCH[2]}
		mkdir -m 0700 -p ~/.ssh
		if ! ssh-keyscan "$HOST" >>~/.ssh/known_hosts; then
			print_fail "git_repo_clone" "Failed to scan key for $HOST ($URL)"
			return 2
		fi
	fi
}

git_ssh_identity () {
	URL=${1:-}

	IDENTITY=

	if [[ "$URL" =~ ^(ssh://|git@)([^/]+)[/:]([^/]+)/(.+)\.git$ ]]; then
		ORG=${BASH_REMATCH[3]}
		REPO=${BASH_REMATCH[4]}

		KEY_REPO=${REPO^^}
		KEY_REPO=${KEY_REPO//-/_}
		IDENTITY=
		SSH_KEY_VARNAME=SSH_DEPLOY_PRIVKEY_${KEY_REPO}
		if [ -n "${!SSH_KEY_VARNAME}" ]; then
			IDENTITY=$HOME/.ssh/id_${ORG}_${REPO}
			mkdir -m 0700 -p $HOME/.ssh
			echo "${!SSH_KEY_VARNAME}" >$IDENTITY
			chmod 600 $IDENTITY
		fi
	fi

	echo $IDENTITY
}

git_repo_clone()
{
	URL=${1:-}
	DIR=${2:-}
	ARGS=${3:-}

	if [ -z "$DIR" ]; then
		print_fail "git_repo_clone" "Target directory argument missing for $URL"
		return 1
	fi
	if [ -d $DIR ]; then
		print_fail "git_repo_clone" "Target directory $DIR already exists"
		return 1
	fi

	git_ssh_keyscan "$URL"
	IDENTITY=$(git_ssh_identity "$URL")

	if [ -n "$IDENTITY" ]; then
		export GIT_SSH_COMMAND="ssh -o IdentitiesOnly=yes -i $IDENTITY"
	fi
	if ! git clone --recursive $ARGS $URL $DIR; then
		if [ -n "$IDENTITY" ]; then unset GIT_SSH_COMMAND; fi
		print_fail "git_repo_clone" "Failed to clone from $URL"
		return 2
	fi
	if [ -n "$IDENTITY" ]; then unset GIT_SSH_COMMAND; fi

	return 0
}

# Fetch from remote and integrate changes in current directory
git_repo_pull()
{
		OP=${1:-}
		if [ "$OP" != "no-fetch" ]; then
				git_repo_fetch
		fi
		
		local REMOTE_URL=$(git config --get remote.origin.url)
		local REPO_DIR=$(pwd)
		local REPO_NAME=$(basename $REPO_DIR)

		local REPO_CHANGED=$(git_repo_changed) || return $?

		case "$REPO_CHANGED" in
				pull)
					git_ssh_keyscan "$REMOTE_URL"
					IDENTITY=$(git_ssh_identity "$REMOTE_URL")

					if [ -n "$IDENTITY" ]; then
						export GIT_SSH_COMMAND="ssh -o IdentitiesOnly=yes -i $IDENTITY"
					fi
					echo "Pulling changes from remote"
					if ! git pull --ff-only ; then
						if [ -n "$IDENTITY" ]; then unset GIT_SSH_COMMAND; fi
						print_fail "git_repo_pull" "failed to update $(dirname $(pwd))"
						return 1
					fi
					if [ -n "$IDENTITY" ]; then unset GIT_SSH_COMMAND; fi
					;;
				push)
						print_fail "git_pull" "Repository $REPO_NAME contains local changes to push"
						;;
				diverged)
						print_fail "git_pull" "Repository $REPO_NAME local and remove have diverged"
						;;
				up-to-date)
						echo "Repository $REPO_NAME is already up to date"
						;;
				*)
						echo "Unknown repository state for $REPO_NAME"
						return 1
						;;
		esac

		return 0
}

git_repo_pull_cond_build()
{
		local REPO_DIR=$(pwd)
		local REPO_NAME=$(basename $REPO_DIR/)
		local REPO_CHANGED=$(git_repo_changed) || exit $?
		local BUILD_CMD=${1:-make -j4 all gui}
		
		case "$REPO_CHANGED" in
				pull)
						git_repo_pull no-fetch || exit $?
						echo -e "\n\n"
						echo "*** Building software ***"
						$BUILD_CMD
						;;
				push)
						print_fail "git_repo_pull_cond_build" "Repository $REPO_NAME contains local changes to push"
						return 1
						;;
				diverged)
						print_fail "git_repo_pull_cond_build" "Repository $REPO_NAME local and remove have diverged"
						return 1
						;;
				up-to-date)
						echo "Repository $REPO_NAME is already up to date"
						;;
				*)
						echo "Unknown repository state for $REPO_NAME"
						return 1
						;;
		esac

		return 0
}


# Get commit hash of script/current directory repository
git_repo_hash()
{
		local GIT_HASH=$(git rev-parse HEAD)
		if [ -z "$GIT_HASH" ]; then
				print_fail "git_repo_hash" "determining HEAD hash failed"
				return 1
		fi
		echo $GIT_HASH
		return 0
}

print_sechead()
{
		local SECHEAD={$1:-}
		local HALF_WIDTH_L=$(((64-${#SECHEAD})/2))
		local HALF_WIDTH_R=$HALF_WIDTH_L
		if (( ( HALF_WIDTH_L * 2 + ${#SECHEAD} ) < 64 )); then
				HALF_WIDTH_R=$(( HALF_WIDTH_R + 1 ))
		fi
	  
		echo -e "\n"
		echo -e "\033[1;32m##########################################################################\033[0m"
		printf "\\033[1;32m###\\033[0m %${HALF_WIDTH_L}s \\033[1;33m%s\\033[0m %${HALF_WIDTH_R}s \\033[1;32m###\\033[0m\n" "" "$SECHEAD" ""
		echo -e "\033[1;32m##########################################################################\033[0m"
		echo
}

patch_apply()
{
		local P=${1:-}

		if patch -p1 -N --dry-run --quiet <$P 2>&1 >/dev/null; then
			if patch -p1 -N <$P ; then
				return 0
			else
				return 2
			fi
		else
			return 1
		fi

		return 3
}

patch_apply_all()
{
		local PATCHES=$@
		local APPLIED_ANY=1

		for p in $PATCHES; do
				PATCH_NAME=$(basename $p)

				if ! patch_check_applied $p; then
						echo "--- Applying patch $PATCH_NAME"
						if patch_apply $p; then
								APPLIED_ANY=0
						else
								print_fail "$SCRIPT_NAME" "Failed to apply patch $PATCH_NAME"
						fi
				else
						echo "Patch $PATCH_NAME already applied"
				fi
		done

		return $APPLIED_ANY
}

patch_check_applied()
{
		local P=${1:-}
		if patch -p1 -N --dry-run --quiet <$P 2>&1 >/dev/null; then
				return 1
		else
				return 0
		fi
}

patch_check_needs_clean()
{
		local F=${1%.patch}.flags

		if [ -e "$F" ]; then
				while read -r flags_line; do
						if [ "$flags_line" == "clean" ]; then
								return 0
						fi
				done <"$F"
		fi

		return 1
}

patch_check_needs_clean_all()
{
		local PATCHES=$@
		local APPLIED_ANY=1
		
		for p in $PATCHES; do
				if patch_check_needs_clean $p; then
						return 0
				fi
		done

		return 1
}

# Determine number of available CPU cores
num_proc()
{
	local NPROC=1
	if type -p nproc >/dev/null; then
		# Linux
		NPROC=$(nproc)
	elif sysctl -n hw.ncpu >/dev/null 2>&1; then
		# FreeBSD
		NPROC=$(sysctl -n hw.ncpu)
	fi
	echo $NPROC
}

# Determine GNU Make executable
find_gmake()
{
	MAKE=make
	if type -p gmake >/dev/null; then
		MAKE=gmake
	fi
	echo $MAKE
}
