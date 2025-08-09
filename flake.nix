{
  description = "ZMK/Zephyr dev flake";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    systems.url = "github:nix-systems/default";
    flake-utils = {
      url = "github:numtide/flake-utils";
      inputs.systems.follows = "systems";
    };
  };

  outputs =
    { nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
      in
      {
        devShells.default = pkgs.mkShell {
          packages = with pkgs; [
            python3Full
            python313Packages.west
            # From requirements-base.txt
            python313Packages.pyelftools
            python313Packages.pyyaml
            python313Packages.pykwalify
            # python313Packages.canopen # Try to update later, it's proken for now. https://github.com/NixOS/nixpkgs/pull/423735#pullrequestreview-3036010982
            python313Packages.packaging
            python313Packages.patool
            python313Packages.psutil
            python313Packages.pylink-square
            python313Packages.pyserial
            python313Packages.requests
            python313Packages.semver
            python313Packages.tqdm
            python313Packages.reuse
            python313Packages.anytree
            python313Packages.intelhex
            # From requirements-extras.txt
            python313Packages.gitpython
            gitlint
            python313Packages.junit2html
            python313Packages.lpc-checksum
            python313Packages.spsdk
            python313Packages.pillow
            python313Packages.pygithub
            python313Packages.graphviz
          ];
        };
      }
    );
}
