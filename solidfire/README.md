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

Getting all those pieces working right can take some time...


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

We currently build from a branch named "solidfire/4.11".  The 4.11 of
course corresponds to the kernel.org branch that our kernel is based on,
in this case "linux-4.11.y"

As the kernel.org maintainers make new stable releases on this branch,
we'll merge those tags into our branch (or possibly rebase our branch
on top of those tags).

In the future when we have to maintain multiple kernels (for example,
one fore Neon and one for Sodium) we might insert the target element
or element version number into our branch and tag names, but that's
future work.
