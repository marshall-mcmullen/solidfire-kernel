# What are all the pieces

The Ember kernel consists of three parts that all live in different
git repositories.


## The kernel sources

The main part is of course the Linux source code.  This lives here:

https://bitbucket.org/solidfire/solidfire-kernel

Each "official" version of the ember kernel has a git tag like
"v4.11.7-solidfire1".  Our tags are based on the upstream (kernel.org) tag
that our kernel is based on, in this case "v4.11.7", with "-solidfireX"
appended, where "X" is an integer that indicates the solidfire version.


## The ebuilds

The solidfire-portage repo contains the kernel ebuilds:

https://bitbucket.org/solidfire/solidfire-portage

The kernel ebuilds live in the sys-kernel/solidfire-sources directory.

Because of limitations of gentoo ebuild version numbers, the ebuild
version numbers look like "4.11.7.1s", where the ".1s" corresponds to the
"-solidfire1" part of the git tag.


## Ember

The Ember repo ties everything together.  It is here:

https://bitbucket.org/solidfire/ember

The "base" flavor of ember (in flavors/base) provides the kernel to all
ember flavors.

Ember uses the `genkernel` tool to build the kernel.  Genkernel gets
its kernel configuration from `flavors/base/conf/kernel.exports`, and
the kernel config is in `flavors/base/config/kernel.config`.


# The build process

There are two ways to build the kernel: slow official builds and rapid
developer builds.


## Official builds

Official builds happen as part of the normal (Ember
build)[https://bitbucket.org/solidfire/ember#markdown-header-build-process]
when Ember's genkernel config matches the available solidfire-sources
ebuilds in the solidfire-portage snapshot, and the ebuilds reference
valid tags in the solidfire-kernel repo.

Getting all those pieces working right takes some careful work:

1.  Push the kernel sources to the solidfire-kernel repo.  Official Ember
    kernels should have a tag identifying the commit, developer builds
    do not need it.

2.  In the solidfire-portage repo, add a new ebuild to
    sys-kernel/solidfire-sources with the EGIT_COMMIT pointing at
    the commit you want to build.  Update the manifest.  Push to the
    "ember" branch.

3.  Tell Jenkins to make a new solidfire-portage snapshot:
    http://pw-jenkins.pw.solidfire.net:8080/job/ember-snapshots/

4.  In the ember repo, update flavors/base/conf/kernel.exports (and
    kernel.config, if needed).

5.  "make element-iso"


## Quick developer builds

Ember ships by default with the full kernel sources installed in
`/usr/src/linux`.  These sources can be built as is, or connected to a
git repo of your choice, modified, and rebuilt.

There are two options for building the ember kernel: genkernel and
manual builds.


### genkernel

The genkernel way is nice because it manages building the kernel &
modules, installing everything in the right places, and building the
initrd.

Genkernel can do many different things depending on how it's invoked,
but here's an illustrative and useful example:

`genkernel --kerneldir=/usr/src/linux --kernname=my-test --kernel-config=/usr/src/.config-my-test --makeopts=-j25 all`


### Manual builds

This is just invoking the kernel build system directly and setting things
up by hand.  If this appeals to you, you probably know what to do already.

`make -j25 && make install && make modules_install`


# Kernel branching

We build our kernels from branches with names that begin with
"solidfire/".  The string after that is the first two digits of the
upstream kernel that our kernel is based on.  For example, the Ember
kernel based on kernel.org's "linux-4.11.y" branch is in a branch named
"solidfire/4.11".

As the kernel.org maintainers make new stable releases on this branch,
we'll merge those tags into our branch (or possibly rebase our branch
on top of those tags).

In the future when we have to maintain multiple kernels (for example, one
for Neon and one for Sodium) we might insert the target element or element
version number into our branch and tag names, but that's future work.


# Updating Ember's kernel to a new kernel.org version

kernel.org's kernel moves relentlessly forward, and we choose when we
want to pick up their new changes.


## Updating to a new minor version

Updating our kernel to a new kernel.org minor version, for example
going from 4.9.10 to 4.9.30, is easy: just merge the new tag into the
appropriate solidfire branch (in this case solidfire/4.9).

(Alternatively, we should consider using rebase here.)


## Updating to a new major version

Updating our kernel to a new kernel.org major version, for example going
from 4.11.7 to 4.12.3, is a little more work.

kernel.org keeps major versions of stable kernels in separate branches
(in this example, linux-4.11.y and linux-4.12.y) that don't share their
recent history, so a simple merge won't work well.

We use this process to get around the problems:

1. Identify the kernel.org tag that our current kernel is based on,
   in this example that's "v4.11.7".

2. Identify the kernel.org tag that we want our new kernel to be based
   on, in this example that's "v4.12.3".

3. Create the new solidfire branch at the new tag, in this case that's
   done with a command like this: `git branch solidfire/4.12 v4.12.3`

4. Check out the new branch: `git checkout solidfire/4.12`

5. Merge the old kernel.org tag into our new branch using the "ours"
   merge strategy: `git merge -s ours v4.11.7`

   The resulting tree is identical to the branch before the merge,
   but the histories are tied together in a way git understands now.

   You can verify that this step was done correctly by seeing that there's
   no diff from v4.12.3 to solidfire/4.12 right at the merge commit.

6. Merge the old solidfire branch into the new solidfire branch in the
   usual way: `git merge solidfire/4.11`

7. Tag it & push it.

Then follow the standard build process for getting the new kernel
into ember: In solidfire-portage, make an ebuild, commit it, tag it,
and push it.  Make a solidfire-portage snapshot.  In ember, update the
kernel source package version number commit it and push it.
