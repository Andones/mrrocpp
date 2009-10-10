<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Euler XYZ window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<!-- signals handling file .cc-->
<xsl:template name="irp6.xyz_euler_zyz.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="xyz_euler_zyz" select="xyz_euler_zyz"/>
<xsl:text>

extern "C"
{
	void on_arrow_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
		</xsl:text><xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.arrow">
    		<xsl:with-param name="xyz_euler_zyz" select="$xyz_euler_zyz"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>
	}
	
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz (GtkButton* button, gpointer userdata)
		{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	</xsl:text><xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.read.1">
    		<xsl:with-param name="xyz_euler_zyz" select="$xyz_euler_zyz"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>
 		
		if (robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text></xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"></xsl:when><xsl:otherwise><xsl:text>->ecp</xsl:text></xsl:otherwise></xsl:choose><xsl:text>->get_EDP_pid()!=-1)
		{
			if (state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised) // Czy robot jest zsynchronizowany?
			{
			    try {
    				robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->read_xyz_euler_zyz(</xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_e); // Odczyt polozenia walow silnikow
       					
    </xsl:text><xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.read.2">
        				<xsl:with-param name="xyz_euler_zyz" select="$xyz_euler_zyz"/>
        				<xsl:with-param name="name" select="$name"/>
    					<xsl:with-param name="i" select="1"/>
     				</xsl:call-template><xsl:text>		
     				
    				for (int i = 0; i &lt; </xsl:text><xsl:value-of select="$xyz_euler_zyz" /><xsl:text>; i++)
    				</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_e[i] = </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_e[i];
    		    }
    		    
    		    </xsl:text><xsl:call-template name="catch" /><xsl:text>
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout &lt;&lt; "Robot is not synchronized" &lt;&lt; std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	</xsl:text><xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.execute.1">
    		<xsl:with-param name="xyz_euler_zyz" select="$xyz_euler_zyz"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>    

		if (robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text></xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"></xsl:when><xsl:otherwise><xsl:text>->ecp</xsl:text></xsl:otherwise></xsl:choose><xsl:text>->get_EDP_pid()!=-1)
		{
			if (state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised) {
	</xsl:text><xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.execute.2">
    		<xsl:with-param name="xyz_euler_zyz" select="$xyz_euler_zyz"/>
    		<xsl:with-param name="name" select="$name"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>    
			
			robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->move_xyz_euler_zyz(</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_e);
			}
			 if (state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised) {
	</xsl:text><xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.execute.3">
    		<xsl:with-param name="xyz_euler_zyz" select="$xyz_euler_zyz"/>
    		<xsl:with-param name="name" select="$name"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>  
			 }
		}
		on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz (button, userdata);

	}
	
	void on_export_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
     
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	</xsl:text><xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.execute.1">
    	<xsl:with-param name="xyz_euler_zyz" select="$xyz_euler_zyz"/>
		<xsl:with-param name="i" select="1"/>
		<xsl:with-param name="name" select="$name"/>
 	</xsl:call-template><xsl:text>
 		sprintf(buffer, "edp_</xsl:text><xsl:value-of select="$name" /><xsl:text> XYZ EULER ZYZ position </xsl:text><xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.export.1">
    		<xsl:with-param name="xyz_euler_zyz" select="$xyz_euler_zyz"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>" 
 		</xsl:text><xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.export.2">
    		<xsl:with-param name="xyz_euler_zyz" select="$xyz_euler_zyz"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>);
 		  
 		gtk_entry_set_text (entryConsole, buffer);  
	}
	
	void on_import_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
        
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
		
		</xsl:text><xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.import.1">
    		<xsl:with-param name="xyz_euler_zyz" select="$xyz_euler_zyz"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>  
 	}
	
</xsl:text>
		<xsl:call-template name="for.each.edp.irp6.xyz_euler_zyz.signals.cc">
    		<xsl:with-param name="xyz_euler_zyz" select="$xyz_euler_zyz"/>
    		<xsl:with-param name="fullName" select="$fullName"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>
<xsl:text>
}
</xsl:text>
</xsl:template>

<!-- irp6 axis xyz handling signals .cc repeatable part -->
<xsl:template name="irp6.xyz_euler_zyz.repeat.signals.cc.arrow">
<xsl:param name="xyz_euler_zyz"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz">
	<xsl:text>
        GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, atof(gtk_entry_get_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>)));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz">
          <xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.arrow">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz">
                  <xsl:value-of select="$xyz_euler_zyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_euler_zyz.repeat.signals.cc.import.1">
<xsl:param name="xyz_euler_zyz"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz">
	<xsl:text>
 	    GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, atof(gtk_entry_get_text(entryConsole)));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz">
          <xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.import.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz">
                  <xsl:value-of select="$xyz_euler_zyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_euler_zyz.repeat.signals.cc.execute.1">
<xsl:param name="xyz_euler_zyz"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz">
	<xsl:text>	GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
 	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz">
          <xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.execute.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz">
                  <xsl:value-of select="$xyz_euler_zyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_euler_zyz.repeat.signals.cc.export.1">
<xsl:param name="xyz_euler_zyz"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz">
	<xsl:text> %f</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz">
          <xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.export.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz">
                  <xsl:value-of select="$xyz_euler_zyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_euler_zyz.repeat.signals.cc.export.2">
<xsl:param name="xyz_euler_zyz"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz">
	<xsl:text>, gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>)</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz">
          <xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.export.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz">
                  <xsl:value-of select="$xyz_euler_zyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_euler_zyz.repeat.signals.cc.read.1">
<xsl:param name="xyz_euler_zyz"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz">
	<xsl:text>	GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz">
          <xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.read.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz">
                  <xsl:value-of select="$xyz_euler_zyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_euler_zyz.repeat.signals.cc.read.2">
<xsl:param name="xyz_euler_zyz"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz">
	<xsl:text>					snprintf (buf, sizeof(buf), "%.3f", </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_e[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]);
					gtk_entry_set_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, buf);
					</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_e[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>] = </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_e[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>];				
</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz">
          <xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.read.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz">
                  <xsl:value-of select="$xyz_euler_zyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_euler_zyz.repeat.signals.cc.execute.2">
<xsl:param name="xyz_euler_zyz"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz">
	<xsl:text>			</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_e[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>] = gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>);
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz">
          <xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.execute.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz">
                  <xsl:value-of select="$xyz_euler_zyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_euler_zyz.repeat.signals.cc.execute.3">
<xsl:param name="xyz_euler_zyz"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz">
	<xsl:text>			gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, </xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_e[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]);
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz">
          <xsl:call-template name="irp6.xyz_euler_zyz.repeat.signals.cc.execute.3">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz">
                  <xsl:value-of select="$xyz_euler_zyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- handling signals .cc repeatable part -->
<xsl:template name="for.each.edp.irp6.xyz_euler_zyz.signals.cc">
<xsl:param name="xyz_euler_zyz"/>
<xsl:param name="fullName"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_euler_zyz">
<xsl:text>
	void on_button</xsl:text><xsl:value-of select="($i*2)-1" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>) - gtk_spin_button_get_value(spinbuttonDown1_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>));
 		
 		on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz (button, userdata);
 	} 
	
	void on_button</xsl:text><xsl:value-of select="($i*2)" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>) + gtk_spin_button_get_value(spinbuttonDown1_xyz_euler_zyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>));
 		
 		on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz (button, userdata);
 	}   
</xsl:text>
    </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_euler_zyz">
          <xsl:call-template name="for.each.edp.irp6.xyz_euler_zyz.signals.cc">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_euler_zyz">
                  <xsl:value-of select="$xyz_euler_zyz"/>
              </xsl:with-param>
              <xsl:with-param name="fullName">
                  <xsl:value-of select="$fullName"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

</xsl:stylesheet>
