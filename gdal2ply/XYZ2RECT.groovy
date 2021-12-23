#!/usr/bin/env groovy
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.nio.file.Paths
import java.util.concurrent.atomic.AtomicLong

println()
println(this.class.getSimpleName() + ' ' + args.findAll({ !it.isBlank() }).join(' '))

def cli = new CliBuilder(
        usage: "${this.class.getSimpleName()} -i <input file> -pw <pixel width> [<options>]"
        , header: 'Options:'
        , width: 80
        , stopAtNonOption: false
)
cli.with {
    i(longOpt: 'input', args: 1, argName: 'file', 'Input file')
    o(longOpt: 'output', args: 1, argName: 'file', 'Output file')
    p(longOpt: 'pixel-width', args: 1, argName: 'number', 'Pixel Width')
    r(longOpt: 'rect', args: 1, argName: 'rectangle (x,y,width,height)', 'Clipping rectangle')
    h(longOpt: 'help', 'Print this help text')
}

def options = cli.parse(args)
assert options

if (options.h) {
    cli.usage()
    System.exit(0)
}

if (!options.i) {
    println('No input file specified')
    System.exit(1)
}

if (!options.p) {
    println('No pixel width specified')
    System.exit(1)
}

def input = Paths.get((String) options.i).toAbsolutePath()
println('In: ' + input)

def output = (options.o) ? Paths.get((String) options.o).toAbsolutePath() : input.getParent().resolve("${input.getFileName()}.rect")
println('Out: ' + output)
println()

def rect = null as Long[]
if (options.r) {
    rect = Arrays.stream((options.r as String).trim().split(',')).map({ Long.valueOf(it) }).limit(4).toArray({ new Long[it] })
    if (rect.length != 4) {
        println("rect needs exact 4 values, but only ${rect.length} values found")
        System.exit(1)
    }
}

println('Filter ...')
if (!rect) {
    Files.copy(input, output)
} else {

    def width = Long.valueOf((String) options.p)

    Files.newBufferedWriter(output, StandardCharsets.UTF_8).withCloseable({ writer ->

        Files.newBufferedReader(input, StandardCharsets.UTF_8).withCloseable({ inStream ->

            def idx = 0L
            def line = '' as String
            print(line)
            while ((line = inStream.readLine()) != null) {
                long y = (long) ((long) idx / (long) width)
                long x = (long) idx % (long) width

                if (x >= rect[0] && x < (rect[0] + rect[2]) && y >= rect[1] && y < (rect[1] + rect[3])) {
                    writer.writeLine(line)
                }

                idx++
            }

        })

    })

}

println('...Done')
