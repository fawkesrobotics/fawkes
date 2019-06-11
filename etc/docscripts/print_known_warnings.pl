#!/usr/bin/perl

use File::Find;
use File::Spec;
use Fcntl ':mode';
use Getopt::Long;
use List::Util qw(any);

my @known_files = ();
my $known_files_file;
my $warning_file;
my @warning_lines = ();

my @paths = ();
GetOptions("w=s" => \$warning_file, "k=s" => \$known_files_file);

die("ERROR: warning files must be specified.") unless defined $warning_file;
die("ERROR: known files must be specified.") unless defined $known_files_file;


open(my $fh, '<', $known_files_file) or die "Could not open file with git-known files";
chomp(@known_files = <$fh>);
close($fh);

open(my $fh, '<', $warning_file) or die "Could not open file with warnings";
chomp(@warning_lines = <$fh>);
close($fh);

my $should_print = 0;
foreach my $line (@warning_lines){
  if($line =~ /(^[^[:space:]:;,<>?\"*|\\]+)/){ # get lines with a file
    if(any { $_ =~ $1 } @known_files){
      $should_print = 1;
    } else {
      $should_print = 0;
    }
  } 
  if($should_print != 0) {
    print("$line","\n");
  }
}
