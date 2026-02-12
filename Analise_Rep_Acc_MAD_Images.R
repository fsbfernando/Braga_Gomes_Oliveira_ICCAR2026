# Full pipeline:
# 1) Read raw data (Experimento_v2.csv)
# 2) Compute errors and err_eucl (2D: X,Y)
# 3) Remove outliers (MAD) by Block x AprilTag
# 4) Aggregate to 45 rows (ISO): Euclidean Accuracy and Repeatability
# 5) Fit log mixed model: Response ~ Method * Illumination + (1 | AprilTag)
# 6) Export PNG boxplots + PNG ANOVA tables (2 analyses) + TXT summary report

# Packages
pkgs <- c("readr", "dplyr", "lme4", "lmerTest", "car")
to_install <- pkgs[!(pkgs %in% rownames(installed.packages()))]
if (length(to_install) > 0) install.packages(to_install)

library(readr)
library(dplyr)
library(lme4)
library(lmerTest)
library(car)

# MAD function (outliers)
remove_outliers_mad <- function(x, k = 3.5) {
  med <- median(x, na.rm = TRUE)
  mad_val <- mad(x, constant = 1, na.rm = TRUE)
  if (is.na(mad_val) || mad_val == 0) {
    return(rep(TRUE, length(x)))
  }
  abs(x - med) <= k * mad_val
}

# Read raw data
CSV_FILE <- "Experimento_v2.csv"
df <- read_csv(CSV_FILE, show_col_types = FALSE)

# Ensure correct types
df <- df %>%
  mutate(
    Bloco      = factor(Bloco),
    Metodo     = factor(Metodo),
    Iluminacao = factor(Iluminacao),
    AprilTag   = factor(AprilTag)
  )

# Compute errors and 2D Euclidean error
df <- df %>%
  mutate(
    err_x = Est_X - Real_X,
    err_y = Est_Y - Real_Y,
    err_z = Est_Z - Real_Z,
    err_eucl = sqrt(err_x^2 + err_y^2)
  )

# Remove outliers by Block x AprilTag
df_clean <- df %>%
  group_by(Bloco, AprilTag) %>%
  mutate(
    keep = remove_outliers_mad(err_eucl, k = 3.5)
  ) %>%
  filter(keep) %>%
  ungroup() %>%
  dplyr::select(-keep)

cat("\nOutlier removal (by Block x AprilTag)\n")

removed_report <- df %>%
  count(Bloco, AprilTag, name = "N_before") %>%
  left_join(df_clean %>% count(Bloco, AprilTag, name = "N_after"),
            by = c("Bloco", "AprilTag")) %>%
  mutate(
    N_after = ifelse(is.na(N_after), 0L, N_after),
    removed = N_before - N_after
  )

print(removed_report)
cat("\nTotal before :", nrow(df), "\n")
cat("Total after :", nrow(df_clean), "\n")

# Aggregate to 45 rows (ISO eq. 12–17)
df_summary <- df_clean %>%
  group_by(Bloco, Metodo, Iluminacao, AprilTag) %>%
  summarise(
    N = n(),
    mean_est_x = mean(Est_X, na.rm = TRUE),
    mean_est_y = mean(Est_Y, na.rm = TRUE),
    gt_x = first(Real_X),
    gt_y = first(Real_Y),
    Acuracia_Euclidiana = sqrt((mean_est_x - gt_x)^2 + (mean_est_y - gt_y)^2),
    l_bar = {
      lk <- sqrt((Est_X - mean_est_x)^2 + (Est_Y - mean_est_y)^2)
      mean(lk, na.rm = TRUE)
    },
    S_l = {
      lk <- sqrt((Est_X - mean_est_x)^2 + (Est_Y - mean_est_y)^2)
      sd(lk, na.rm = TRUE)
    },
    Repetibilidade_Euclidiana = l_bar + 3.0 * S_l,
    .groups = "drop"
  ) %>%
  dplyr::select(Bloco, Metodo, Iluminacao, AprilTag, N,
                Acuracia_Euclidiana, Repetibilidade_Euclidiana)

cat("\nAggregated summary (expected 45 rows)\n")
cat("Rows:", nrow(df_summary), "\n")
print(head(df_summary))

# Log transform and mixed model
eps <- 1e-12
df_summary <- df_summary %>%
  mutate(
    log_Acuracia = log(Acuracia_Euclidiana + eps),
    log_Repetibilidade = log(Repetibilidade_Euclidiana + eps)
  )

cat("\nCheck: count by Method, Illumination, AprilTag\n")
print(df_summary %>% count(Metodo, Iluminacao, AprilTag))

# Model + ANOVA + Shapiro
run_mixed_anova <- function(data, response_col, title = "") {
  cat("\n", title, "\n")
  cat("Model: ", response_col, " ~ Metodo * Iluminacao + (1 | AprilTag)\n", sep = "")
  
  fml <- as.formula(paste0(response_col, " ~ Metodo * Iluminacao + (1 | AprilTag)"))
  fit <- lmer(fml, data = data, REML = TRUE)
  
  anova_tbl <- car::Anova(fit, type = 2)
  cat("\nANOVA (Type II)\n")
  print(anova_tbl)
  
  res <- residuals(fit)
  sh <- shapiro.test(res)
  cat("\nShapiro-Wilk (residuals)\n")
  print(sh)
  
  cat("\nModel summary (lmer)\n")
  print(summary(fit))
  
  list(model = fit, anova = anova_tbl, shapiro = sh)
}

# Run both analyses (log scale)
out_acc_log <- run_mixed_anova(
  data = df_summary,
  response_col = "log_Acuracia",
  title = "ANALYSIS 1 — ACCURACY (LOG SCALE) | outliers removed"
)

out_rep_log <- run_mixed_anova(
  data = df_summary,
  response_col = "log_Repetibilidade",
  title = "ANALYSIS 2 — REPEATABILITY (LOG SCALE) | outliers removed"
)

# Residual QQ-plots
par(mfrow = c(1, 2))
qqnorm(residuals(out_acc_log$model), main = "Residual QQ-plot (log Accuracy)")
qqline(residuals(out_acc_log$model))
qqnorm(residuals(out_rep_log$model), main = "Residual QQ-plot (log Repeatability)")
qqline(residuals(out_rep_log$model))
par(mfrow = c(1, 1))

# Output folder and exports
outdir <- "out_iso_anova"
if (!dir.exists(outdir)) dir.create(outdir)

alpha <- 0.05

sig_flag <- function(p) {
  if (is.na(p)) return("NA")
  if (p < alpha) "Significant" else "Not significant"
}

export_outputs <- function(df_summary, out_obj, response_col, pretty_name, file_prefix) {
  
  png(filename = file.path(outdir, paste0("boxplot_iluminacao_", file_prefix, ".png")),
      width = 700, height = 420, res = 120)
  boxplot(
    df_summary[[response_col]] ~ df_summary$Iluminacao,
    xlab = "Illumination level",
    ylab = "log(err_norm)",
    main = ""
  )
  dev.off()
  
  png(filename = file.path(outdir, paste0("boxplot_metodo_", file_prefix, ".png")),
      width = 700, height = 420, res = 120)
  boxplot(
    df_summary[[response_col]] ~ df_summary$Metodo,
    xlab = "Method",
    ylab = "log(err_norm)",
    main = ""
  )
  dev.off()
  
  an_tbl <- anova(out_obj$model)
  tab_lines <- capture.output(print(an_tbl, digits = 6))
  
  png(filename = file.path(outdir, paste0("anova_table_", file_prefix, ".png")),
      width = 900, height = 320, res = 120)
  par(mar = c(0, 0, 0, 0))
  plot.new()
  text(
    x = 0, y = 1,
    labels = paste(tab_lines, collapse = "\n"),
    adj = c(0, 1),
    family = "mono",
    cex = 0.95
  )
  dev.off()
  
  get_p <- function(term) {
    if (!(term %in% rownames(an_tbl))) return(NA_real_)
    an_tbl[term, "Pr(>F)"]
  }
  get_f <- function(term) {
    if (!(term %in% rownames(an_tbl))) return(NA_real_)
    an_tbl[term, "F.value"]
  }
  get_df1 <- function(term) {
    if (!(term %in% rownames(an_tbl))) return(NA_real_)
    an_tbl[term, "NumDF"]
  }
  get_df2 <- function(term) {
    if (!(term %in% rownames(an_tbl))) return(NA_real_)
    an_tbl[term, "DenDF"]
  }
  
  p_m <- get_p("Metodo")
  p_l <- get_p("Iluminacao")
  p_i <- get_p("Metodo:Iluminacao")
  
  f_m <- get_f("Metodo")
  f_l <- get_f("Iluminacao")
  f_i <- get_f("Metodo:Iluminacao")
  
  df1_m <- get_df1("Metodo"); df2_m <- get_df2("Metodo")
  df1_l <- get_df1("Iluminacao"); df2_l <- get_df2("Iluminacao")
  df1_i <- get_df1("Metodo:Iluminacao"); df2_i <- get_df2("Metodo:Iluminacao")
  
  sh_p <- out_obj$shapiro$p.value
  
  lines <- c(
    paste0("REPORT — ", pretty_name),
    paste0("Model: ", response_col, " ~ Metodo * Iluminacao + (1 | AprilTag)"),
    "",
    "Shapiro-Wilk (mixed-model residuals):",
    paste0("  p = ", format(sh_p, digits = 6), "  -> ", sig_flag(sh_p)),
    "",
    "ANOVA (lmerTest::anova):",
    paste0("  Metodo:            F(", df1_m, ", ", df2_m, ") = ", format(f_m, digits = 6),
           " | p = ", format(p_m, digits = 6), " -> ", sig_flag(p_m)),
    paste0("  Iluminacao:        F(", df1_l, ", ", df2_l, ") = ", format(f_l, digits = 6),
           " | p = ", format(p_l, digits = 6), " -> ", sig_flag(p_l)),
    paste0("  Metodo×Iluminacao: F(", df1_i, ", ", df2_i, ") = ", format(f_i, digits = 6),
           " | p = ", format(p_i, digits = 6), " -> ", sig_flag(p_i)),
    "",
    paste0("Files: boxplot_iluminacao_", file_prefix, ".png"),
    paste0("       boxplot_metodo_", file_prefix, ".png"),
    paste0("       anova_table_", file_prefix, ".png"),
    ""
  )
  
  return(lines)
}

report_acc <- export_outputs(
  df_summary = df_summary,
  out_obj = out_acc_log,
  response_col = "log_Acuracia",
  pretty_name = "ACCURACY (LOG SCALE)",
  file_prefix = "log_acuracia"
)

report_rep <- export_outputs(
  df_summary = df_summary,
  out_obj = out_rep_log,
  response_col = "log_Repetibilidade",
  pretty_name = "REPEATABILITY (LOG SCALE)",
  file_prefix = "log_repetibilidade"
)

report_all <- c(
  "SHORT REPORT — ANOVA + BOX PLOTS + TABLES",
  paste0("DateTime: ", as.character(Sys.time())),
  paste0("Alpha: ", alpha),
  "",
  report_acc,
  report_rep
)

writeLines(report_all, con = file.path(outdir, "relatorio_resumido.txt"))

cat("\nOutputs saved in:", outdir, "\n")
cat("PNG: 4 boxplots + 2 ANOVA tables\n")
cat("TXT: relatorio_resumido.txt\n")
