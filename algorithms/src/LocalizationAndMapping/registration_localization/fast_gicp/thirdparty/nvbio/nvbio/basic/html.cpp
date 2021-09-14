/*
 * nvbio
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the NVIDIA CORPORATION nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <nvbio/basic/html.h>
#include <stdarg.h>
#include <string.h>

namespace nvbio {
namespace html {

const char* style()
{
    return
/*
stats th also goes with:

background-color:#82A416;
color:#000000;
border:1px solid #729404;
*/
"body\n\
{\n\
background-color:#252525;\n\
}\n\
span statnum\n\
{\n\
	float:left;\n\
    width:60px;\n\
}\n\
span statbar\n\
{\n\
	background-color:#AADD44;\n\
	color:#AADD44;\n\
	solid:#AADD44;\n\
	border:1px solid #555555;\n\
	float:left;\n\
	display:inline-block;\n\
	margin-left:5px;\n\
}\n\
table.params\n\
{\n\
font-family:\"Calibri\", \"Courier New\", \"Trebuchet MS\", Arial, Helvetica, sans-serif;\n\
width:84%;\n\
margin-left:8%;\n\
margin-right:8%;\n\
border-collapse:collapse;\n\
}\n\
table.params caption\n\
{\n\
font-family:\"Courier New\", \"Trebuchet MS\", Arial, Helvetica, sans-serif;\n\
font-size:1.0em;\n\
color:#FFFFFF;\n\
background-color:#000000;\n\
padding:4px 7px 4px 7px;\n\
}\n\
table.params tr a       { color:#FFFFFF; }\n\
table.params tr a:hover { color:#99DD66; }\n\
table.params th\n\
{\n\
width:50%;\n\
font-size:0.9em;\n\
text-align:left;\n\
padding-top:5px;\n\
padding-bottom:4px;\n\
background-color:#AAAAAA;\n\
color:#FFFFFF;\n\
border:1px solid #999999;\n\
}\n\
table.params td, th\n\
{\n\
font-size:0.8em;\n\
border:1px solid #CBCBCB;\n\
padding:3px 7px 2px 7px;\n\
background-color:#EBEBEB;\n\
}\n\
table.params tr.alt td\n\
{\n\
color:#000000;\n\
background-color:#DADADA;\n\
}\n\
table.stats\n\
{\n\
font-family:\"Calibri\", \"Courier New\", \"Trebuchet MS\", Arial, Helvetica, sans-serif;\n\
width:84%;\n\
margin-left:8%;\n\
margin-right:8%;\n\
border-collapse:collapse;\n\
}\n\
table.stats tr a       { color:#DDDDFF; }\n\
table.stats tr a:hover { color:#99DD66; }\n\
table.stats td, th\n\
{\n\
font-size:0.8em;\n\
border:1px solid #BBBBBB;\n\
padding:3px 7px 2px 7px;\n\
background-color:#DDDDDD;\n\
}\n\
table.stats td.small\n\
{\n\
font-size:0.7em;\n\
border:1px solid #BBBBBB;\n\
padding:3px 7px 2px 7px;\n\
}\n\
table.stats td.smallpink\n\
{\n\
font-size:0.7em;\n\
padding:3px 7px 2px 7px;\n\
color:#000000;\n\
background-color:#FFE5D5;\n\
border:1px solid #BB9988;\n\
}\n\
table.stats th\n\
{\n\
width:2%;\n\
font-size:0.9em;\n\
text-align:left;\n\
padding-top:5px;\n\
padding-bottom:4px;\n\
background-color:#444444;\n\
color:#DDDDDD;\n\
border:1px solid #383838;\n\
}\n\
table.stats caption\n\
{\n\
font-family:\"Courier New\", \"Trebuchet MS\", Arial, Helvetica, sans-serif;\n\
font-size:1.0em;\n\
color:#AADD44;\n\
height:24px;\n\
background-color:#000000;\n\
padding:12px 7px 4px 7px;\n\
}\n\
table.stats tr.alt td\n\
{\n\
color:#000000;\n\
background-color:#EAEAEA;\n\
}\n\
table.stats tr.alt td.small\n\
{\n\
font-size:0.7em;\n\
color:#000000;\n\
background-color:#EAEAEA;\n\
}\n\
table.stats tr.alt td.smallpink\n\
{\n\
font-size:0.7em;\n\
color:#000000;\n\
background-color:#FFD7C3;\n\
border:1px solid #BB9988;\n\
}\n\
table.stats tr.alt td.red\n\
{\n\
color:#000000;\n\
background-color:#FFAABB;\n\
border:1px solid #BB6655;\n\
}\n\
table.stats tr.alt td.green\n\
{\n\
color:#000000;\n\
background-color:#D6FF93;\n\
border:1px solid #A6CC83;\n\
}\n\
table.stats tr.alt td.pink\n\
{\n\
color:#000000;\n\
background-color:#FFD7C3;\n\
border:1px solid #BB9988;\n\
}\n\
table.stats tr.alt td.azure\n\
{\n\
color:#000000;\n\
background-color:#99DDFF;\n\
}\n\
table.stats tr.alt td.gray\n\
{\n\
color:#000000;\n\
background-color:#BBBBBB;\n\
border:1px solid #999999;\n\
}\n\
table.stats tr.alt td.orange\n\
{\n\
color:#000000;\n\
background-color:#FFBB55;\n\
border:1px solid #DD9911;\n\
}\n\
table.stats tr.alt td.yellow\n\
{\n\
color:#000000;\n\
background-color:#FFFF77;\n\
border:1px solid #CCCC33;\n\
}\n\
table.stats td.red\n\
{\n\
color:#000000;\n\
background-color:#FF9988;\n\
border:1px solid #BB6655;\n\
}\n\
table.stats td.green\n\
{\n\
color:#000000;\n\
background-color:#DCFF9A;\n\
border:1px solid #AACC89;\n\
}\n\
table.stats td.pink\n\
{\n\
color:#000000;\n\
background-color:#FFE5D5;\n\
border:1px solid #BB9988;\n\
}\n\
table.stats td.azure\n\
{\n\
color:#000000;\n\
background-color:#99DDFF;\n\
}\n\
table.stats tr td.gray\n\
{\n\
color:#000000;\n\
background-color:#BBBBBB;\n\
border:1px solid #999999;\n\
}\n\
table.stats td.orange\n\
{\n\
color:#000000;\n\
background-color:#FFBB66;\n\
border:1px solid #DD9911;\n\
}\n\
table.stats td.yellow\n\
{\n\
color:#000000;\n\
background-color:#FFFF99;\n\
border:1px solid #CCCC33;\n\
}";
#if 0
    // params
    "table.params\n"
    "{\n"
    "font-family:\"Trebuchet MS\", Arial, Helvetica, sans-serif;\n"
    "width:84%;\n"
    "margin-left:8%;\n"
    "margin-right:8%;\n"
    "border-collapse:collapse;\n"
    "}\n"
    "table.params caption\n"
    "{\n"
    "font-family:\"Trebuchet MS\", Arial, Helvetica, sans-serif;\n"
    "font-size:1.1em;\n"
    "color:#FFFFFF;\n"
    "background-color:#555555;\n"
    "padding:4px 7px 4px 7px;\n"
    "}\n"
    "table.params tr a\n"
    "{\n"
	"color:#FFFFFF;\n"
    "}\n"
    "table.params th\n"
    "{\n"
    "font-size:0.9em;\n"
    "text-align:left;\n"
    "padding-top:5px;\n"
    "padding-bottom:4px;\n"
    "background-color:#AAAAAA;\n"
    "color:#FFFFFF;\n"
    "border:1px solid #999999;\n"
    "}\n"
    "table.params td, th\n"
    "{\n"
    "font-size:0.8em;\n"
    "border:1px solid #CBCBCB;\n"
    "padding:3px 7px 2px 7px;\n"
    "background-color:#EBEBEB;\n"
    "}\n"
    "table.params tr.alt td\n"
    "{\n"
    "color:#000000;\n"
    "background-color:#DADADA;\n"
    "}\n"
    // stats
    "table.stats\n"
    "{\n"
    "font-family:\"Trebuchet MS\", Arial, Helvetica, sans-serif;\n"
    "width:84%;\n"
    "margin-left:8%;\n"
    "margin-right:8%;\n"
    "border-collapse:collapse;\n"
    "}\n"
    "table.stats tr a\n"
    "{\n"
	"color:#FFFFFF;\n"
    "}\n"
    "table.stats td, th\n"
    "{\n"
    "font-size:0.8em;\n"
    "border:1px solid #98bf21;\n"
    "padding:3px 7px 2px 7px;\n"
    "}\n"
    "table.stats td.small\n"
    "{\n"
    "font-size:0.7em;\n"
    "border:1px solid #98bf21;\n"
    "padding:3px 7px 2px 7px;\n"
    "}\n"
    "table.stats td.smallpink\n"
    "{\n"
    "font-size:0.7em;\n"
    "padding:3px 7px 2px 7px;\n"
    "color:#000000;\n"
    "background-color:#FFCCBB;\n"
    "border:1px solid #BB9988;\n"
    "}\n"
    "table.stats th\n"
    "{\n"
    "font-size:0.9em;\n"
    "text-align:left;\n"
    "padding-top:5px;\n"
    "padding-bottom:4px;\n"
    "background-color:#A7C942;\n"
    "color:#ffffff;\n"
    "}\n"
    "table.stats caption\n"
    "{\n"
    "font-family:\"Trebuchet MS\", Arial, Helvetica, sans-serif;\n"
    "font-size:1.1em;\n"
    "color:#FFFFFF;\n"
    "background-color:#6688FF;\n"
    "padding:4px 7px 4px 7px;\n"
    "}\n"
    "table.stats tr.alt td\n"
    "{\n"
    "color:#000000;\n"
    "background-color:#EAF2D3;\n"
    "}\n"
    "table.stats tr.alt td.small\n"
    "{\n"
    "font-size:0.7em;\n"
    "color:#000000;\n"
    "background-color:#EAF2D3;\n"
    "}\n"
    "table.stats tr.alt td.smallpink\n"
    "{\n"
    "font-size:0.7em;\n"
    "color:#000000;\n"
    "background-color:#FFBBAA;\n"
    "border:1px solid #BB9988;\n"
    "}\n"
    "table.stats tr.alt td.red\n"
    "{\n"
    "color:#000000;\n"
    "background-color:#FFAA99;\n"
    "border:1px solid #BB6655;\n"
    "}\n"
    "table.stats tr.alt td.pink\n"
    "{\n"
    "color:#000000;\n"
    "background-color:#FFBBAA;\n"
    "border:1px solid #BB9988;\n"
    "}\n"
    "table.stats tr.alt td.azure\n"
    "{\n"
    "color:#000000;\n"
    "background-color:#99DDFF;\n"
    "}\n"
    "table.stats tr.alt td.gray\n"
    "{\n"
    "color:#000000;\n"
    "background-color:#BBBBBB;\n"
    "border:1px solid #999999;\n"
    "}\n"
    "table.stats tr.alt td.orange\n"
    "{\n"
    "color:#000000;\n"
    "background-color:#FFBB33;\n"
    "}\n"
    "table.stats tr.alt td.yellow\n"
    "{\n"
    "color:#000000;\n"
    "background-color:#FFFF44;\n"
    "}\n"
    "table.stats td.red\n"
    "{\n"
    "color:#000000;\n"
    "background-color:#FF9988;\n"
    "border:1px solid #BB6655;\n"
    "}\n"
    "table.stats td.pink\n"
    "{\n"
    "color:#000000;\n"
    "background-color:#FFCCBB;\n"
    "border:1px solid #BB9988;\n"
    "}\n"
    "table.stats td.azure\n"
    "{\n"
    "color:#000000;\n"
    "background-color:#99DDFF;\n"
    "}\n"
    "table.stats tr td.gray\n"
    "{\n"
    "color:#000000;\n"
    "background-color:#BBBBBB;\n"
    "border:1px solid #999999;\n"
    "}\n"
    "table.stats td.orange\n"
    "{\n"
    "color:#000000;\n"
    "background-color:#FFBB33;\n"
    "}\n"
    "table.stats td.yellow\n"
    "{\n"
    "color:#000000;\n"
    "background-color:#FFFF44;\n"
    "}";
#endif
}

void html_begin(FILE* output)
{
    fprintf( output, "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0 Transitional//EN\">\n" );
    fprintf( output, "<html lang=\"en\">\n" );
}
void html_end(FILE* output)
{
    fprintf( output, "</html>\n" );
}
void header(FILE* output, const char* title, const char* css, const char* meta)
{
    fprintf( output, "<head>\n" );
    fprintf( output, "<meta http-equiv=\"Content-Type\" content=\"text/html; charset=ISO-8859-1\">\n" );
    if (meta != NULL)
        fprintf( output, meta );
    fprintf( output, "<title>%s</title>\n", title );
    const char* css_end = css + strlen( css )-4;
    if (strcmp( css_end, ".css" ) == 0)
        fprintf( output, "<link rel=\"stylesheet\" href=\"%s\" type=\"text/css\">\n", css );
    else
        fprintf( output, "<style>\n%s\n</style>\n", css );
    fprintf( output, "</head>\n ");
}
void body_begin(FILE* output)
{
    fprintf( output, "<body>\n" );
}
void body_end(FILE* output)
{
    fprintf( output, "</body>\n" );
}
void table_begin(FILE* output, const char* id, const char* cls, const char* caption)
{
    fprintf( output, "<table id = \"%s\" class = \"%s\">\n", id, cls );
    fprintf( output, "<caption>%s</caption>\n", caption );
}
void table_end(FILE* output)
{
    fprintf( output, "</table>\n" );
}
void tr_begin(FILE* output, ...)
{
    va_list args;
    va_start( args, output );

    fprintf( output, "<tr" );
    while (1)
    {
        const char* key = va_arg( args, const char* );
        if (key == NULL)
            break;

        const char* value = va_arg( args, const char* );

        fprintf( output, " %s = \"%s\"", key, value );
    }
    fprintf( output, ">\n" );

    va_end(args);
}
void tr_end(FILE* output)
{
    fprintf( output, "</tr>\n" );
}
void th(FILE* output, const char* name, ...)
{
    va_list args;
    va_start( args, name );

    fprintf( output, "<th" );
    while (1)
    {
        const char* key = va_arg( args, const char* );
        if (key == NULL)
            break;

        const char* value = va_arg( args, const char* );

        fprintf( output, " %s = \"%s\"", key, value );
    }
    fprintf( output, ">" );
    fprintf( output, "%s</th>\n", name );

    va_end(args);
}
void td(FILE* output, const char* name, ...)
{
    va_list args;
    va_start( args, name );

    fprintf( output, "<td" );
    while (1)
    {
        const char* key = va_arg( args, const char* );
        if (key == NULL)
            break;

        const char* value = va_arg( args, const char* );

        fprintf( output, " %s = \"%s\"", key, value );
    }
    fprintf( output, ">" );
    fprintf( output, "%s</td>\n", name );

    va_end(args);
}
void vtr_begin(FILE* output, va_list args)
{
    fprintf( output, "<tr" );
    while (1)
    {
        const char* key = va_arg( args, const char* );
        if (key == NULL)
            break;

        const char* value = va_arg( args, const char* );

        fprintf( output, " %s = \"%s\"", key, value );
    }
    fprintf( output, ">\n" );
}
void vth(FILE* output, const char* name, va_list args)
{
    fprintf( output, "<th" );
    while (1)
    {
        const char* key = va_arg( args, const char* );
        if (key == NULL)
            break;

        const char* value = va_arg( args, const char* );

        fprintf( output, " %s = \"%s\"", key, value );
    }
    fprintf( output, ">" );
    fprintf( output, "%s</th>\n", name );
}
void vtd(FILE* output, const char* name, va_list args)
{
    fprintf( output, "<td" );
    while (1)
    {
        const char* key = va_arg( args, const char* );
        if (key == NULL)
            break;

        const char* value = va_arg( args, const char* );

        fprintf( output, " %s = \"%s\"", key, value );
    }
    fprintf( output, ">" );
    fprintf( output, "%s</td>\n", name );
}
void vth(FILE* output, const Formatting formatted, va_list args)
{
    fprintf( output, "<th" );
    while (1)
    {
        const char* key = va_arg( args, const char* );
        if (key == NULL)
            break;

        const char* value = va_arg( args, const char* );

        fprintf( output, " %s = \"%s\"", key, value );
    }
    fprintf( output, ">" );
    const char* format = va_arg( args, const char* );
    vfprintf( output, format, args );
    fprintf( output, "</th>\n" );
}
void vtd(FILE* output, const Formatting formatted, va_list args)
{
    fprintf( output, "<td" );
    while (1)
    {
        const char* key = va_arg( args, const char* );
        if (key == NULL)
            break;

        const char* value = va_arg( args, const char* );

        fprintf( output, " %s = \"%s\"", key, value );
    }
    fprintf( output, ">" );
    const char* format = va_arg( args, const char* );
    vfprintf( output, format, args );
    fprintf( output, "</td>\n" );
}

tr_object::tr_object(FILE* output, ...) :
    m_output( output )
{
    va_list args;
    va_start(args, output);
    vtr_begin( output, args );
    va_end(args);
}
th_object::th_object(FILE* output, const char* name, ...) :
    m_output( output )
{
    va_list args;
    va_start(args, name);
    vth( output, name, args );
    va_end(args);
}
th_object::th_object(FILE* output, const Formatting formatted, ...) :
    m_output( output )
{
    va_list args;
    va_start(args, formatted);
    vth( output, formatted, args );
    va_end(args);
}
/*
th_object::td_object(FILE* output, const uint32 value, ...) :
    m_output( output )
{
    va_list args;
    va_start(args, value);
    char name[1024];
    sprintf( name, "%u", value );
    vth( output, name, args );
    va_end(args);
}
th_object::td_object(FILE* output, const int32 value, ...) :
    m_output( output )
{
    va_list args;
    va_start(args, value);
    char name[1024];
    sprintf( name, "%i", value );
    vth( output, name, args );
    va_end(args);
}
th_object::td_object(FILE* output, const float value, ...) :
    m_output( output )
{
    va_list args;
    va_start(args, value);
    char name[1024];
    sprintf( name, "%f", value );
    vth( output, name, args );
    va_end(args);
}
th_object::td_object(FILE* output, const float value, const uint32 prec, ...) :
    m_output( output )
{
    va_list args;
    va_start(args, value);
    char format[1024];
    char name[1024];
    sprintf( format, "%u%%f", prec );
    sprintf( name, format, value );
    vth( output, name, args );
    va_end(args);
}
*/
td_object::td_object(FILE* output, const char* name, ...) :
    m_output( output )
{
    va_list args;
    va_start(args, name);
    vtd( output, name, args );
    va_end(args);
}
td_object::td_object(FILE* output, const Formatting formatted, ...) :
    m_output( output )
{
    va_list args;
    va_start(args, formatted);
    vtd( output, formatted, args );
    va_end(args);
}
/*
td_object::td_object(FILE* output, const uint32 value, ...) :
    m_output( output )
{
    va_list args;
    va_start(args, value);
    char name[1024];
    sprintf( name, "%u", value );
    vtd( output, name, args );
    va_end(args);
}
td_object::td_object(FILE* output, const int32 value, ...) :
    m_output( output )
{
    va_list args;
    va_start(args, value);
    char name[1024];
    sprintf( name, "%i", value );
    vtd( output, name, args );
    va_end(args);
}
td_object::td_object(FILE* output, const float value, ...) :
    m_output( output )
{
    va_list args;
    va_start(args, value);
    char name[1024];
    sprintf( name, "%f", value );
    vtd( output, name, args );
    va_end(args);
}
td_object::td_object(FILE* output, const float value, const uint32 prec, ...) :
    m_output( output )
{
    va_list args;
    va_start(args, value);
    char format[1024];
    char name[1024];
    sprintf( format, "%u%%f", prec );
    sprintf( name, format, value );
    vtd( output, name, args );
    va_end(args);
}
*/
void test()
{
    FILE* output = fopen( "test.html", "w" );

    // let's write our HTML
    {
        html_object html( output );
        {
            header_object hd( output, "Test", style() );
            {
                body_object body( output );
                {
                    table_object table( output, "my_stats", "stats", "stats" );
                    {
                        tr_object tr( output, NULL );
                        th_object( output, "", NULL );
                        th_object( output, "stats1", NULL );
                        th_object( output, "stats2", NULL );
                        th_object( output, "stats3", NULL );
                    }
                    {
                        tr_object tr( output, NULL );
                        th_object( output, "first", NULL );
                    }
                    {
                        tr_object tr( output, "class", "alt", NULL );
                        th_object( output, "first", NULL );
                        td_object( output, "0.5", NULL );
                        td_object( output, "0.1", NULL );
                        td_object( output, "0.7", NULL );
                    }
                    {
                        tr_object tr( output, NULL );
                        th_object( output, "second", NULL );
                    }
                    {
                        tr_object tr( output, "class", "alt", NULL );
                        th_object( output, "second", NULL );
                        td_object( output, "0.25", NULL );
                        td_object( output, "0.3", NULL );
                        td_object( output, "0.9", NULL );
                    }
                    {
                        tr_object tr( output, NULL );
                        th_object( output, "third", NULL );
                    }
                    {
                        tr_object tr( output, "class", "alt", NULL );
                        th_object( output, "third", NULL );
                        td_object( output, "0.3", NULL );
                        td_object( output, "0.72", NULL );
                        td_object( output, "0.1", NULL );
                    }
                }
            }
        }
    }
    fclose( output );
}

} // namespace html
} // namespace nvbio
