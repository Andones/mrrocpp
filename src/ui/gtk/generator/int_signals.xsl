<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Inc window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<!-- signals handling file .cc-->
<xsl:template name="irp6.int.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="irp6EDPNumber" select="irp6EDPNumber"/>
<xsl:document method="text" href="../signals/{$name}_int_widget.cc">


<xsl:text>
#include &lt;iostream&gt;
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;
#include "ui_model.h"
#include "</xsl:text><xsl:value-of select="$name" /><xsl:text>_int_widget.h"


edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_int::edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_int(ui_widget_entry &amp;entry) 
{
}

static edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_int *int_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;


extern "C"
{
	void on_arrow_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_int (GtkButton* button, gpointer userdata)
	{
		std::cout &lt;&lt; "skopiuj wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> int" &lt;&lt; std::endl;
	}
	
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_int (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "wczytaj wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> int" &lt;&lt; std::endl;
	}
	
	void on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_int (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "Execute move dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> int" &lt;&lt; std::endl;
	}
	
	void on_export_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_int (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "Export dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> int" &lt;&lt; std::endl;
	}
	
	void on_import_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_int (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "Import dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> int" &lt;&lt; std::endl;
	}
	
	
	void ui_widget_init(ui_widget_entry &amp;entry) 
	{
		int_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_int(entry);
		fprintf(stderr, "widget %s loaded\n", __FILE__);
	}

	void ui_widget_unload(void) 
	{
		if (int_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>) 
		{
			delete int_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
		}
		fprintf(stderr, "widget %s unloaded\n", __FILE__);
	}
	
</xsl:text>
		<xsl:call-template name="for.each.edp.irp6.int.signals.cc">
    			<xsl:with-param name="irp6EDPNumber" select="$irp6EDPNumber"/>
    			<xsl:with-param name="fullName" select="$fullName"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template>
<xsl:text>
}
</xsl:text>
</xsl:document>
<xsl:call-template name="irp6.int.main.signals.h"/>
</xsl:template>



<!-- handling signals .cc repeatable part -->
<xsl:template name="for.each.edp.irp6.int.signals.cc">
<xsl:param name="irp6EDPNumber"/>
<xsl:param name="fullName"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $irp6EDPNumber*2">
<xsl:text>
	void on_button</xsl:text><xsl:value-of select="$i" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_int (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "Button</xsl:text><xsl:value-of select="$i" /><xsl:text> dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> int" &lt;&lt; std::endl;
	}
    
</xsl:text>
    </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $irp6EDPNumber*2">
          <xsl:call-template name="for.each.edp.irp6.int.signals.cc">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="irp6EDPNumber">
                  <xsl:value-of select="$irp6EDPNumber"/>
              </xsl:with-param>
              <xsl:with-param name="fullName">
                  <xsl:value-of select="$fullName"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>



<!-- signals handling file .h-->
<xsl:template name="irp6.int.main.signals.h" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../signals/{$name}_int_widget.h">

<xsl:text>
#ifndef __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_INT
#define __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_INT

#include &lt;iostream&gt;
#include &lt;vector&gt;

#include &lt;gtk/gtkbuilder.h&gt;
#include &lt;gtk/gtk.h&gt;

class edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_int
{
	public:

		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_int(ui_widget_entry &amp;entry);
		~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_int();


};

#endif /* __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_INT */
</xsl:text>
</xsl:document>
</xsl:template>

</xsl:stylesheet>
