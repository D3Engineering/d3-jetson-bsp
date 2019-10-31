#!/usr/bin/env perl
# d3-select-cameras: Permit a user to edit configuration information stored
# in a EEPROM.  Named because the typical use case is setting the
# active_overlays based on which cameras are connected.
# NOTE: Runs on Unix-like systems only - not portable to Windows.
# See COPYRIGHT statement below.

# ==========================================================================
# Imports and definitions

use 5.010001;    # for autodie
use strict;
use warnings;
use version 0.77;

our $VERSION = qv('0.0.3');

use constant { true => !!1, false => !!0 };

# Assist the user if some packages are missing
use Config;
use File::Spec;
use Module::Load::Conditional 'can_load';

BEGIN {
    # Check that we can load modules
    my ( $have_all_deps, $fn ) = can_load(
        modules => {
            'Curses::UI'               => undef,
            'Data::Hexdumper'          => undef,
            'Encode::Locale'           => '1.04',
            'File::Find::Object::Rule' => undef
        }
    );

    # Check that we have perldoc.  This is more complicated than it should
    # be since Ubuntu /usr/bin/perldoc is a stub shell script if perl-doc
    # is not installed.  Therefore, check the first line to see if it's perl.
    # Thanks to Pod::Usage for file locations.
    if ( -x ( $fn = File::Spec->catfile( $Config{scriptdirexp}, 'perldoc' ) ) ) {
        $have_all_deps &&= do { local ( @ARGV, $. ) = ($fn); <> }
            =~ m/^#!.*\bperl\b/;
    } elsif ( -x ( $fn = File::Spec->catfile( $Config{scriptdir}, 'perldoc' ) ) ) {
        $have_all_deps &&= do { local ( @ARGV, $. ) = ($fn); <> }
            =~ m/^#!.*\bperl\b/;
    } else {
        $have_all_deps = false;    # no perldoc => missing perl-doc package
    }

    unless ($have_all_deps) {
        die "Can't find all dependencies - please run:\n"
            . '    sudo apt install -y perl-doc'
            . ' libcurses-ui-perl'
            . ' libdata-hexdumper-perl'
            . ' libencode-locale-perl'
            . " libfile-find-object-rule-perl\n";
    }
} # BEGIN

use Curses::UI;            # from Ubuntu package libcurses-ui-perl
use Data::Hexdumper ();    # from Ubuntu package libdata-hexdumper-perl
use Encode 2.15;           # for bytes2str/str2bytes.
                           # Perl 5.10.1+ has Encode 2.35+ in core, so we don't
                           # need the Ubuntu libencode-perl package.
use Encode::Locale qw(decode_argv);  # from Ubuntu package libencode-locale-perl
use English;
use File::Find::Object::Rule;    # from package libfile-find-object-rule-perl
use Getopt::Long 2.33 qw(GetOptionsFromArray :config gnu_getopt auto_version);
use Pod::Usage;                  # Relies on Ubuntu package perl-doc

# Global
my $VERBOSE = false;             # Verbosity (-v)

sub log_if_verbose;    # Prints to stderr if $VERBOSE is true.  Defined below.

decode_argv();         # Convert bytes->Unicode chars in @ARGV

exit main(@ARGV);

# ==========================================================================
# Main program

# Takes command-line arguments.  Returns a shell exit code, or dies.
sub main {
    my @argv = @_;

    # === Get command-line options ===

    my $dump_only    = false;    # Whether to dump the EEPROM to stdout
    my $eeprom_bytes = 128;      # Size of data to read/write from the EEPROM (-s)
    my $eeprom_filename;         # A specific filename to use
    my $force_given   = false;   # Whether -f was specified
    my $force         = false;   # Skip prompts (-f)
    my $list_devices  = false;   # Whether to list the available devices
    my $no_new_data   = false;   # New data is empty (--none)
    my $should_reboot = false;   # Whether to reboot after writing (-r)

    my $help  = false;           # --help, -h, -?
    my $usage = false;           # --usage
    my $man   = false;           # Whether to show full documentation

    GetOptionsFromArray(
        \@argv,
        'd|dump'       => \$dump_only,
        'e|eeprom=s'   => \$eeprom_filename,
        'f|force'      => \$force_given,
        'h|?|help'     => \$help,
        'list-devices' => \$list_devices,
        'man'          => \$man,
        'none'         => \$no_new_data,
        'r|reboot'     => \$should_reboot,
        's|size=i'     => \$eeprom_bytes,
        'usage'        => \$usage,
        'v|verbose'    => \$VERBOSE

            # Getopt::Long handles --version
    ) or pod2usage( -verbose => 0, -exitval => 2 );

    # Print help if the user asked for it
    pod2usage( -verbose => 0, -exitval => 0 ) if $usage;
    pod2usage( -verbose => 1, -exitval => 0 ) if $help;
    pod2usage( -verbose => 2, -exitval => 0 ) if $man;

    # Figure out what we're going to do

    my $is_interactive = -t STDIN && -t STDOUT && -t STDERR;
    $dump_only ||= ( !-t STDOUT );    # s-c > foo => dump; don't write
    $force = $force_given;
    $force   ||= ( !-t STDIN );       # Reading from stdin (s-c < foo) =>
                                      # always force, since we can't prompt.
    $VERBOSE ||= $list_devices;

    # === Sanity checks ===

    die 'Please provide data to be written via the '
        . 'command line or via stdin, but not both'
        if @argv && ( !-t STDIN );

    die '--eeprom and --list-devices are mutually exclusive'
        if $eeprom_filename && $list_devices;

    die 'Please give all the data to write as one command-line argument'
        if @argv > 1;

    die 'I am not configured to write and dump in a single run'
        . ( $force_given ? ' (rerun without -f?)' : '' )
        if $dump_only && ( @argv || $force );

    warn "** Run as root, e.g., 'sudo $0', if you want to write the EEPROM.\n"
        unless $dump_only || $EFFECTIVE_USER_ID == 0;

    # === Find the EEPROM ===

    unless ($eeprom_filename) {
        my $finder  = File::Find::Object::Rule->directory->maxdepth(1)->mindepth(1);
        my @devices = $finder->in('/sys/bus/nvmem/devices');

        unless (@devices) {
            my $string = 'No devices found.';
            if ($list_devices) {
                $string .= "\n";
            } else {
                $string .= <<EOT;
  Please specify the device's filename with -e, e.g.:

    $0 -e /sys/bus/nvmem/devices/42-1337/nvmem
EOT
            }
            die $string;
        }

        if ($list_devices) {
            say join "\n", map { "$_/nvmem" } @devices;
            exit(0);
        }

        log_if_verbose join( "\n", 'Found devices:', @devices );

        die "More than one device - I haven't yet been taught to choose one"
            if @devices != 1;

        $eeprom_filename = "$devices[0]/nvmem";
    }

    unless ( -e $eeprom_filename ) {
        warn "Creating file $eeprom_filename";
        use autodie ':file';
        open my $fh, '>', $eeprom_filename;
        close $fh;
    }

    unless ( -r -w $eeprom_filename ) {
        my $string = "I do not have rw- on $eeprom_filename";
        $string .= ".  (Perhaps try 'sudo $0 ...'?)"
            unless $EFFECTIVE_USER_ID == 0;
        die $string;
    }

    log_if_verbose "Using device $eeprom_filename";
    log_if_verbose "Locale encoding is $Encode::Locale::ENCODING_LOCALE";

    # === Read the existing value ===

    my $existing_bytes = read_eeprom( $eeprom_filename, $eeprom_bytes );

    hexdump( 'Existing data:', $existing_bytes ) if $VERBOSE || $is_interactive;

    if ($dump_only) { # s-c > foo --- dump the raw data and exit
        assert( binmode(STDOUT), 'Could not set stdout to binary' );
        log_if_verbose 'Dumping EEPROM contents';
        print $existing_bytes;    # print => no trailing newline
        exit(0);

    } elsif ( $is_interactive && !@argv && !$no_new_data ) {

        # Give users a chance to see the hex data before the input box opens
        unless ( prompt_is_y('Proceed (Y/n)?') ) {
            log_if_verbose 'Exiting';
            exit(0);
        }
    }

    # === Get new data to be written ===

    my $new_text;

    if ($no_new_data) {
        $new_text = '';

    } elsif (@argv) { # Data was provided on the command line
        $new_text = $argv[0];

        # Sanity check - we don't know if the user meant stdin.
        die "I can't understand an argument of '-' - omit it for stdin."
            if $new_text eq '-';

        # No test for the number of bytes, since the user is providing
        # only the desired data.

    } elsif ($is_interactive) { # Prompt the user
        $new_text = prompt_for_data($existing_bytes);

    } else {
        $new_text = read_stdin($eeprom_bytes);
    }

    hexdump( 'New data received', $new_text );

    my $new_bytes = prep_new_data( $new_text, $eeprom_bytes );

    # === Write the new value ===

    hexdump( 'New data to be written:', $new_bytes );

    my $ok_to_write = true;
    if ( $is_interactive && !$force ) {
        $ok_to_write = prompt_is_y('OK to write (y/N)?');
    }

    if ($ok_to_write) {
        write_eeprom_bytes( $eeprom_filename, $new_bytes );
        say STDERR 'Write complete';
    } else {
        say STDERR 'Skipping write';
    }

    # === Reboot ===

    if ( $should_reboot && $ok_to_write ) {
        log_if_verbose 'Rebooting...';
        exec {'shutdown'} qw(shutdown -r now);
        die "Could not reboot: $!";

    } elsif ($should_reboot) {
        log_if_verbose "Skipping reboot, since we didn't write";
    }

    return 0;
} # sub main

# ==========================================================================
# Subroutines

# Read byte data.  Call as read_bytes(filehandle, max byte count);
sub read_bytes {
    my ( $input_fd, $eeprom_bytes ) = @_;
    my $data;

    my $read_status = read( $input_fd, $data, $eeprom_bytes );

    die 'Could not read from input source' unless defined $read_status;

    return $data;
} # sub read_bytes

# Read from the EEPROM.  Call as read_eeprom('filename', max byte count);
sub read_eeprom {
    my ( $filename, $eeprom_bytes ) = @_;
    my $eeprom_fd;

    open $eeprom_fd, '<:raw', $filename
        or die "Could not open $filename for reading";
    log_if_verbose "Reading from $filename";
    my $retval = read_bytes( $eeprom_fd, $eeprom_bytes );

    my $close_result = close $eeprom_fd;
    assert( $close_result, "Could not close handle to $filename" );

    warn "Could not read $eeprom_bytes bytes"
        unless length($retval) == $eeprom_bytes;
    return $retval;
} # sub read_eeprom

# Read from stdin.  Call as read_stdin(max byte count);
sub read_stdin {
    my ($eeprom_bytes) = @_;

    open my $stdin_fd, '<&:raw', \*STDIN
        or die 'Could not open stdin handle for reading';
    assert( binmode($stdin_fd), 'Could not set stdin handle to binary' );

    log_if_verbose 'Reading from stdin';
    my $retval = read_bytes( $stdin_fd, $eeprom_bytes );
    chomp $retval;

    assert( close($stdin_fd), "Could not close handle to stdin" );

    eval { $retval = bytes2str( locale => $retval, Encode::FB_CROAK ); };

    if ($@) {
        warn 'Could not understand the input encoding --- ignore this message'
            . " if that doesn't bother you.\nEncoding error was: $@";
    }

    # No test for the number of bytes read, since it may be coming from echo.
    return $retval;
} # sub read_stdin

# Make sure new data is in the correct format.
# Usage: $new_bytes = prep_new_data($new_text, $eeprom_bytes);
sub prep_new_data {
    my ( $new_bytes, $eeprom_bytes ) = @_;
    my $retval = '' . $new_bytes;    # Make sure it's a copy

    eval { $retval = str2bytes( 'UTF-8', $retval, Encode::FB_CROAK ); };
    if ($@) {
        die "I'm sorry, but I only know how to store Unicode text "
            . "at this time.  Aborting.\nError was: $@";
    }

    assert( !Encode::is_utf8($retval), 'New data: is_utf8 error' );

    # Truncate to length
    if ( length($retval) > $eeprom_bytes - 1 ) {
        say "New data has @{[length $retval]} bytes, but I am only "
            . "configured to write @{[$eeprom_bytes-1]} bytes plus \\0.";
        if ( prompt_is_y('Truncate and proceed (y/N)?') ) {
            $retval = substr $retval, 0, $eeprom_bytes - 1;
        } else {
            log_if_verbose "Exiting";
            exit(0);
        }
    }

    # Zero-pad, including null terminator
    $retval .= "\0" x ( $eeprom_bytes - length($retval) );

    return $retval;
} # sub prep_new_data

# Write the given bytes to the given filename.  Usage:
# write_eeprom('filename', 'new bytes');
# Returns on success; dies on failure.
sub write_eeprom_bytes {
    my ( $filename, $new_bytes ) = @_;
    my $eeprom_fd;

    open $eeprom_fd, '>:raw', $filename
        or die "Could not open $filename for writing";

    assert( !Encode::is_utf8($new_bytes), 'New_bytes: unexpeted Unicode string' );
    print {$eeprom_fd} $new_bytes or die "Could not write to $filename";

    assert( close($eeprom_fd), "Could not close handle to $filename" );
} # sub write_eeprom_bytes

# Print a hex dump of data on stderr.  Call as hexdump('title', 'text to dump');
sub hexdump {
    my ( $title, $data ) = @_;
    die 'Cannot dump undefined data' unless defined $data;

    say STDERR $title if defined $title;
    if ( length($data) > 0 ) {
        print STDERR Data::Hexdumper::hexdump(
            data              => $data,
            suppress_warnings => true
        );
    } else {
        say STDERR "<no data>";
    }
} # sub hexdump

# Prompt the user for new data.  Usage:
# $new_string = prompt_for_data('existing data bytes');
sub prompt_for_data {
    my ($existing_bytes) = @_;

    my $existing_text =
        eval { bytes2str( 'UTF-8' => $existing_bytes, Encode::FB_CROAK ); };

    if ($@) {
        warn 'Could not decode existing text as UTF-8.'
            . "  Ignore this warning if that doesn't bother you!\n ";
        $existing_text = '';
    }

    # Chop trailing \0 before prompting
    my $zero_idx = index( $existing_text, "\0" );
    if ( $zero_idx >= 0 ) {
        $existing_text = substr( $existing_text, 0, $zero_idx );
    }

    # Prompt the user.  NOTE: Curses::UI operates in bytes, not chars,
    # and doesn't handle multibyte.  See
    # https://rt.cpan.org/Public/Bug/Display.html?id=56695 .
    my $cui = Curses::UI->new( -color_support => 1, -clear_on_exit => 1 );

    my $new_bytes = $cui->question(
        -question => "Enter an active_overlays string (e.g., camA_0,camB_1)\n"
            . "(Ctl+U to clear, Ctl+Z to undo)\n"
            . 'For empty data, clear the line and tab to OK '
            . 'before hitting Enter.',
        -answer => str2bytes( locale => $existing_text, Encode::FB_CROAK )
    );

    # Get out of the interactive screen
    $cui->leave_curses;
    undef $cui;

    if ( !defined $new_bytes ) {
        log_if_verbose 'Cancel pressed (or Enter hit on empty line) - exiting';
        exit(0);
    }

    my $new_text = eval { bytes2str( locale => $new_bytes, Encode::FB_CROAK ); };
    if ($@) {
        warn 'Could not understand your entry as locale encoding '
            . "$Encode::Locale::ENCODING_LOCALE.\n"
            . "Ignore this warning if that doesn't bother you!\n ";
        $new_text = $new_bytes;
    }

    return $new_text;
} # sub prompt_for_data

# Print log information if $VERBOSE.  Call as log_if_verbose('something to log')
sub log_if_verbose {
    return unless $VERBOSE;
    say STDERR join ' ', @_;
}

# Prompt for a Y/N input.  Returns true for Y, false for N.  Usage:
# if(prompt_is_y("OK (Y/n)?")) { <do yes actions> } else { <do no actions> }
sub prompt_is_y {
    my $prompt = shift;
    assert( ( $prompt =~ m{Y/n|y/N} ), "Cannot understand prompt $prompt" );
    my $default = !!( $prompt =~ m{Y/n} );    # default true if Y/n, false if y/N

    print $prompt, ' ';
    my $answer = readline STDIN;
    chomp $answer;

    return $default unless $answer;
    return !!( $answer =~ /^y/i );
} # sub prompt_is_y

# Assert conditions that should always hold.  Any assertion violation triggers
# a "contact D3" message.
sub assert {
    my ( $cond, $msg ) = @_;
    die "***Assertion failed - contact D3***\n$msg" unless $cond;
}

# ==========================================================================
# Documentation - nothing after `__END__` will be executed {{{1

__END__

=head1 NAME

d3-select-cameras - tell the system which cameras are connected

=head1 SYNOPSIS

    d3-select-cameras              # Interactive use
    d3-select-cameras < data       # Write EEPROM data from a file
    d3-select-cameras "data"       # Write EEPROM data from the command line
    d3-select-cameras > data       # Read EEPROM data (do not write)

Run C<d3-select-cameras -h> for an option reference, or
C<d3-select-cameras --man> for full documentation.

Reboot after you update the EEPROM to apply changes.

=head1 HOW EEPROM-BASED CONFIGURATION WORKS

This program reads or writes the configuration EEPROM attached to your
system.  D3's drivers then read the EEPROM while the system boots, and
enable the correct camera drivers.

Changes take effect only on reboot.

The configuration EEPROM is exposed via the device tree.  The
C</chosen/d3,config-eeprom@0> node in the device tree must include
a pointer to an NVMEM node attached to the desired EEPROM.  At present,
autodetection is only supported for a single EEPROM configured via NVMEM.
If you have more than one NVMEM node, use the C<-e> argument to specify
the NVMEM file to use, e.g.:

    d3-select-cameras -e /sys/bus/nvmem/devices/42-1337/nvmem

=head1 INPUTS

New data to be written to the EEPROM comes from:

=over

=item *

The first command-line parameter, if one is provided;

=item *

You, via interactive prompt, if you run d3-select-cameras interactively
from a terminal and don't provide a command-line parameter; or

=item *

From stdin, otherwise.

=back

This program rejects C<-> (bare hyphen) as a command-line parameter, since
it might mean "write a C<->" or "write data from stdin".  Just leave off the
command-line argument to write from stdin.

=head1 ARGUMENTS AND OPTIONS

Note: this script will not do a dump-to-file and a write in a single pass
(e.g., C<< d3-select-cameras < foo > bar >>).  Instead, run reads and writes
separately.  This is to avoid confusion regarding which data is being dumped.

=over

=item -d, --dump

Output the contents of the EEPROM to stdout and exit.  This is the default
if stdout is redirected.

=item -e, --eeprom B<file>

Read/write B<file> instead of an automatically-detected EEPROM.
The B<file> will be created if it does not exist.

=item -f, --force

Don't prompt for confirmation before writing.  This is the default if you
are providing the data via stdin.  B<Use at your own risk!>

=item -h, --help, -?

Show basic help.

=item --list-devices

List the devices available on the system, but take no other action.

=item --man

Show detailed help.

=item --none

Empty the EEPROM (so that it indicates no cameras).

=item -r, --reboot

Reboot the system (using C<shutdown -r now>) after a successful write.
You may need to run with C<sudo> for this to work.

=item -s, --size

Set the size of the EEPROM, i.e., the number of EEPROM bytes to read or write.
B<Use at your own risk!>

=item --usage

Show a brief usage reminder.

=item -v, --verbose

Show additional information while running.

=item --version

Show version information.

=back


=head1 AUTHOR

Christopher White (C<cwhite@d3engineering.com>)

=head1 LICENSE AND COPYRIGHT

Copyright (c) 2019 D3 Engineering, LLC.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
more details.

=cut

# }}}1
# perltidy options: -csc -cscp '#' -cscl='sub : BEGIN END' -ce -vmll -ci=4 -trp
# vi: set ft=perl fdm=marker: #
